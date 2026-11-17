// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Holds what a rive-cli run of bench/as logged for every step (HASHLOG) to a reference: the state hash, the hash of
// the whole trajectory record but its build and worker dependent counters, so events, counters, sleep flags and
// probes are held too, and how many callbacks the step had made. The reference is a native recording, or the log of
// the serial port's run for a run with callbacks, which no recording has. node cannot run parallel calls, so the run
// happens in rive-cli and this reads its log.
//   node verify/threads.mjs <recording.bin | serial.log> <run.log> <label> [parts tasks]
//   node verify/threads.mjs <recording.bin> --steps
// parts, a count or default, and tasks, lib/parallel_step's bits, also fail a run whose Parallel.size() is not that,
// that stepped other tasks, that never ran one of them on parts (the narrow phase only if the run ends with
// contacts), or none of whose solver calls got that many parts. It cannot say that every call did, or how much of
// the work the parts other than 0 took: a build that is only slower passes.
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

// What stops a run short of its result or makes the result worthless: the CLI's error lines, this bench's and the
// gates' own, a browser's
const PROBLEM = /trapped|\] error |gave up|BENCH failed|^failed|timed out|RuntimeError|Uncaught/;

// lib/constants.as B2_MAX_WORKERS, the most parts a world steps on
const MAX_WORKERS = 32;

// What a run logged: its problems, each step's hashes and callback count, and the fields of its result line
function readLog( file )
{
	const lines = readFileSync( file, 'utf8' ).split( '\n' );
	const problems = lines.filter( ( l ) => PROBLEM.test( l ) ).map( ( l ) => l.replace( /^.*?\] /, '' ).slice( 0, 120 ) );
	const steps = new Map();
	for ( const line of lines )
	{
		const m = line.match( /HASH step=(\d+) hash=(\d+) record=(\d+) calls=(\d+)/ );
		if ( m === null )
		{
			continue;
		}
		if ( steps.has( Number( m[1] ) ) )
		{
			problems.push( `step ${m[1]} logged twice` );
		}
		steps.set( Number( m[1] ), { hash: Number( m[2] ), record: Number( m[3] ), calls: Number( m[4] ) } );
	}
	const result = lines.find( ( l ) => l.includes( 'BENCH RESULT' ) ) ?? '';
	const field = ( name ) => Number( result.match( new RegExp( ` ${name}=(\\d+)` ) )?.[1] ?? -1 );
	const size = Number( lines.join( '\n' ).match( /BENCH parallel size=(\d+)/ )?.[1] ?? -1 );
	return { problems, steps, field, size };
}

const [file, logFile, label, parts, asked] = process.argv.slice( 2 );
const problems = [];
let reference;
if ( file.endsWith( '.bin' ) )
{
	const bytes = readFileSync( file );
	const records = recordHashes( bytes );
	reference = recordingHashes( bytes ).map( ( hash, step ) => ( { hash, record: records[step], calls: 0 } ) );
}
else
{
	// The serial port's run with callbacks. One that stopped short or never called one would hold nothing.
	const serial = readLog( file );
	reference = Array.from( { length: serial.steps.size }, ( _, step ) => serial.steps.get( step ) );
	problems.push( ...serial.problems.map( ( p ) => `serial run: ${p}` ) );
	if ( serial.field( 'steps' ) !== reference.length )
	{
		problems.push( 'serial run: did not finish' );
	}
	else if ( !( reference.at( -1 )?.calls > 0 ) )
	{
		problems.push( 'serial run: no callback was called' );
	}
}
if ( logFile === '--steps' )
{
	console.log( reference.length );
	process.exit( 0 );
}

const run = readLog( logFile );
problems.push( ...run.problems );

// A run that passes on hashes alone may have stepped serially or on fewer parts than it was given
if ( parts !== undefined )
{
	const { field, size } = run;
	const expected = parts === 'default' ? size : Number( parts );
	const tasks = field( 'parTasks' );
	const callbacks = field( 'callbacks' );
	// A task and the callbacks that keep it on the main lane, as lib/parallel_step.as decides. A run that ends without
	// contacts may never have had any for the narrow phase.
	const collide = field( 'contacts' ) > 0 ? 2 : 0;
	const ran = [ [ 1, 0, 'parSolves' ], [ collide, 2 | 4 | 8, 'parCollides' ], [ 4, 0, 'parFinalizes' ], [ 8, 1, 'parPairs' ] ]
		.filter( ( [ task, blockers ] ) => ( tasks & task ) !== 0 && ( callbacks & blockers ) === 0 );
	const idle = ran.find( ( [ , , name ] ) => field( name ) <= 0 );
	if ( size !== expected || size < 1 )
	{
		problems.push( `Parallel.size() is ${size}` );
	}
	else if ( tasks < 0 )
	{
		problems.push( 'no result line' );
	}
	else if ( asked !== undefined && tasks !== Number( asked ) )
	{
		problems.push( `the run stepped tasks ${tasks}` );
	}
	else if ( idle !== undefined )
	{
		problems.push( `${idle[2]} is ${field( idle[2] )}` );
	}
	else if ( ( tasks & 1 ) !== 0 && field( 'parts' ) !== Math.min( expected, MAX_WORKERS ) )
	{
		problems.push( `the solver ran on ${field( 'parts' )} parts` );
	}
}

let compared = 0;
let divergence = '-';
for ( let step = 0; step < reference.length && divergence === '-'; ++step )
{
	const logged = run.steps.get( step );
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
// A problem says more than the step it stopped the run at
if ( problems.length > 0 )
{
	divergence = problems[0];
}
const pass = compared > 0 && compared === reference.length && problems.length === 0;
console.log( `RESULT ${label} ${compared} ${pass ? 'pass' : 'fail'} ${divergence}` );
process.exit( pass ? 0 : 1 );
