// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Compares what a rive-cli run of bench/as logged for every step (--define=HASHLOG) with the native recording: the
// state hash, and the hash of the whole trajectory record but its build and worker dependent counters, so events,
// counters, sleep flags and probes are held to native too. node cannot run parallel calls, so the run happens in
// rive-cli and this reads its log.
//   node verify/threads.mjs <recording.bin> <run.log> <label>
//   node verify/threads.mjs <recording.bin> --steps
// Prints one RESULT line: RESULT <label> <steps compared> <pass|fail> <first divergence or ->

import { readFileSync } from 'node:fs';
import { recordingHashes } from '../bench/web/core.mjs';

// verify/scenes/record.as: the counters from the stable count on depend on the build and the worker count
const BODY_WORDS = 11;
const STABLE_COUNTERS = 41;

// record.as's Record_Hash over each step of a version 2 trajectory
function recordHashes( bytes )
{
	const w = new Uint32Array( bytes.buffer, bytes.byteOffset, bytes.byteLength >> 2 );
	const counters = w[14];
	const hashes = [];
	let i = 16;
	while ( i < w.length )
	{
		const probeAt = i + 2 + counters + w[i + 1] * BODY_WORDS;
		const end = probeAt + 1 + w[probeAt];
		let h = 2166136261;
		for ( let k = i; k < end; ++k )
		{
			if ( i + 2 + STABLE_COUNTERS <= k && k < i + 2 + counters )
			{
				continue;
			}
			for ( let s = 0; s < 4; ++s )
			{
				h = Math.imul( h ^ ( ( w[k] >>> ( 8 * s ) ) & 0xff ), 16777619 ) >>> 0;
			}
		}
		hashes.push( h );
		i = end;
	}
	return hashes;
}

const [file, logFile, label] = process.argv.slice( 2 );
const bytes = readFileSync( file );
const native = recordingHashes( bytes );
if ( logFile === '--steps' )
{
	console.log( native.length );
	process.exit( 0 );
}
const nativeRecords = recordHashes( bytes );

const lines = readFileSync( logFile, 'utf8' ).split( '\n' );
const run = new Map();
for ( const line of lines )
{
	const m = line.match( /HASH step=(\d+) hash=(\d+) record=(\d+)/ );
	if ( m )
	{
		run.set( Number( m[1] ), { hash: Number( m[2] ), record: Number( m[3] ) } );
	}
}
const problems = lines.filter( ( l ) => /trapped|failed|gave up| error /.test( l ) );

let compared = 0;
let divergence = '-';
for ( let step = 0; step < native.length; ++step )
{
	const logged = run.get( step );
	if ( logged === undefined )
	{
		divergence = `step ${step} missing`;
		break;
	}
	if ( logged.hash !== native[step] )
	{
		divergence = `step ${step} state hash ${logged.hash} native ${native[step]}`;
		break;
	}
	if ( logged.record !== nativeRecords[step] )
	{
		divergence = `step ${step} record hash ${logged.record} native ${nativeRecords[step]}`;
		break;
	}
	compared += 1;
}
if ( problems.length > 0 && divergence === '-' )
{
	divergence = problems[0].replace( /^.*?\] /, '' ).slice( 0, 120 );
}
const pass = compared === native.length && problems.length === 0;
console.log( `RESULT ${label} ${compared} ${pass ? 'pass' : 'fail'} ${divergence}` );
process.exit( pass ? 0 : 1 );
