// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Holds what a rive-cli run of bench/as logged for every step (HASHLOG) to a reference: the state hash, the hash of
// the whole trajectory record but its build and worker dependent counters, so events, counters, sleep flags and
// probes are held too, and how many callbacks the step had made. The reference is a native recording, or the log of
// the serial port's run for a run with callbacks, which no recording has. node cannot run parallel calls, so the run
// happens in rive-cli and this reads its log.
//   node verify/threads.mjs <recording.bin | serial.log> <run.log> <label> [parts]
//   node verify/threads.mjs <recording.bin> --steps
// parts, a count or default, also fails a run whose solver did not run on that many parts.
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

// What a run logged: its lines, and each step's hashes and callback count
function readLog( file )
{
	const lines = readFileSync( file, 'utf8' ).split( '\n' );
	const steps = new Map();
	for ( const line of lines )
	{
		const m = line.match( /HASH step=(\d+) hash=(\d+) record=(\d+) calls=(\d+)/ );
		if ( m )
		{
			steps.set( Number( m[1] ), { hash: Number( m[2] ), record: Number( m[3] ), calls: Number( m[4] ) } );
		}
	}
	return { lines, steps };
}

const [file, logFile, label, parts] = process.argv.slice( 2 );
let reference;
if ( file.endsWith( '.bin' ) )
{
	const bytes = readFileSync( file );
	const records = recordHashes( bytes );
	reference = recordingHashes( bytes ).map( ( hash, step ) => ( { hash, record: records[step], calls: 0 } ) );
}
else
{
	const { steps } = readLog( file );
	reference = Array.from( { length: steps.size }, ( _, step ) => steps.get( step ) );
}
if ( logFile === '--steps' )
{
	console.log( reference.length );
	process.exit( 0 );
}

const { lines, steps } = readLog( logFile );
const problems = lines.filter( ( l ) => /trapped|failed|gave up| error /.test( l ) );

// A run that passes on hashes alone may have stepped serially or on fewer parts than it was given
if ( parts !== undefined )
{
	const result = lines.find( ( l ) => l.includes( 'BENCH RESULT' ) ) ?? '';
	const field = ( name ) => Number( result.match( new RegExp( ` ${name}=(\\d+)` ) )?.[1] ?? -1 );
	const size = Number( lines.join( '\n' ).match( /BENCH parallel size=(\d+)/ )?.[1] ?? -1 );
	const expected = parts === 'default' ? size : Number( parts );
	if ( size !== expected )
	{
		problems.push( `Parallel.size() is ${size}` );
	}
	else if ( field( 'parSolves' ) <= 0 )
	{
		problems.push( 'no solver call ran on parts' );
	}
	else if ( field( 'parts' ) !== expected )
	{
		problems.push( `the solver ran on ${field( 'parts' )} parts` );
	}
}

let compared = 0;
let divergence = '-';
for ( let step = 0; step < reference.length && divergence === '-'; ++step )
{
	const logged = steps.get( step );
	const wanted = reference[step];
	if ( logged === undefined || wanted === undefined )
	{
		divergence = `step ${step} missing`;
		break;
	}
	for ( const [key, name] of [ [ 'hash', 'state hash' ], [ 'record', 'record hash' ], [ 'calls', 'callback calls' ] ] )
	{
		if ( logged[key] !== wanted[key] )
		{
			divergence = `step ${step} ${name} ${logged[key]} reference ${wanted[key]}`;
			break;
		}
	}
	compared += divergence === '-' ? 1 : 0;
}
if ( problems.length > 0 && divergence === '-' )
{
	divergence = problems[0].replace( /^.*?\] /, '' ).slice( 0, 120 );
}
const pass = compared > 0 && compared === reference.length && problems.length === 0;
console.log( `RESULT ${label} ${compared} ${pass ? 'pass' : 'fail'} ${divergence}` );
process.exit( pass ? 0 : 1 );
