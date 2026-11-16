// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The port on WAMR AOT (rive-cli's bench/as, the SIMD artifact) interleaved with the same release build on V8
// and native Box2D, single threaded, every run timed from step 1 on. Appends to a jsonl bench.mjs --table= reads.
//   ROUNDS=3 node bench/web/aot.mjs
// RIVE and RIVE_WAMRC name rive-cli and wamrc, verify/toolchain.sh's by default.

import { execFileSync, spawnSync } from 'node:child_process';
import { appendFileSync, readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { recordingHashes, SCENES } from './core.mjs';

const here = dirname( fileURLToPath( import.meta.url ) );
const root = join( here, '..', '..' );
const project = join( root, 'bench', 'as' );
const toolchain = ( name ) =>
	execFileSync( 'bash', [ '-c', `source "${root}/verify/toolchain.sh" && echo "$${name}"` ], { encoding: 'utf8' } ).trim();
const rive = process.env.RIVE ?? toolchain( 'RIVE' );
const wamrc = process.env.RIVE_WAMRC ?? toolchain( 'RIVE_WAMRC' );
const rounds = Number( process.env.ROUNDS ?? 3 );
// scene:steps; bench/as's SCENE define is the scene's index in verify/scenes
const scenes = ( process.env.SCENES ?? 'large_pyramid:100 many_bodies:100 joint_grid:100 rain:360 smash:120 ' +
	'spinner:300 tumbler:200 pyramid:300' ).split( /\s+/ ).filter( Boolean ).map( ( e ) => e.split( ':' ) );

execFileSync( join( project, 'sync.sh' ) );
// An empty options blob, so a threads run's leftover does not override the defines
writeFileSync( join( project, 'options.txt' ), '' );

mkdirSync( join( here, 'out' ), { recursive: true } );
const stamp = new Date().toISOString().replace( /[-:]/g, '' ).slice( 0, 15 );
const log = join( here, 'out', `aot_${stamp}.jsonl` );
appendFileSync( log, JSON.stringify( { node: process.version, v8: process.versions.v8,
	machine: execFileSync( 'sysctl', [ '-n', 'machdep.cpu.brand_string' ] ).toString().trim() } ) + '\n' );
console.log( `log ${log}` );

function aotRun( scene, steps )
{
	// rive-cli logs to stderr
	const run = spawnSync( rive, [ '.', '--aot', `--bench=${Number( steps ) + 10}`, `--define=SCENE=${SCENES.indexOf( scene )}`,
		`--define=STEPS=${steps}` ], { cwd: project, maxBuffer: 1 << 26, env: { ...process.env, RIVE_WAMRC: wamrc,
		RIVE_WASM_AOT_SYNC: 'o3', RIVE_AOT_CACHE_DIR: join( project, 'out', 'aotcache' ) } } );
	const text = `${run.stdout}${run.stderr}`;
	const tier = /script tier: (.*)/.exec( text )?.[1];
	const line = text.split( '\n' ).find( ( l ) => l.includes( 'BENCH RESULT' ) );
	if ( line === undefined )
	{
		throw new Error( `no BENCH RESULT for ${scene}:\n${text.slice( -2000 )}` );
	}
	const field = ( name ) => Number( new RegExp( ` ${name}=([\\d.]+)` ).exec( line )[1] );
	const reference = recordingHashes( readFileSync( join( root, 'verify', 'scenes', 'out', `${scene}_scalar_w1.bin` ) ) );
	return { impl: 'as_aot', scene, steps: Number( steps ), workers: 1, tier, median: field( 'median' ), min: field( 'min' ),
		mean: field( 'mean' ), compared: 1, mismatch: reference[steps - 1] === field( 'hash' ) ? -1 : steps - 1,
		stages: { step: field( 'step' ), collide: field( 'collide' ), solve: field( 'solve' ) } };
}

for ( let round = 0; round < rounds; ++round )
{
	for ( const [ scene, steps ] of scenes )
	{
		const jobs = [
			() => aotRun( scene, steps ),
			...[ 'native_simd', 'c_simd', 'as_simd' ].map( ( impl ) => () => JSON.parse( execFileSync( process.execPath,
				[ join( here, 'run.mjs' ), impl, scene, steps, '--warmup=1' ], { maxBuffer: 1 << 26 } ).toString().trim()
				.split( '\n' ).pop() ) ),
		];
		for ( let j = 0; j < jobs.length; ++j )
		{
			const load = Number( execFileSync( 'uptime' ).toString().match( /load averages?: ([\d.]+)/ )[1] );
			const result = { round, load, ...jobs[( j + round ) % jobs.length]() };
			appendFileSync( log, JSON.stringify( result ) + '\n' );
			console.log( `round ${round} ${scene} ${result.impl} load ${load} median ${result.median.toFixed( 4 )} ` +
				`${result.mismatch === -1 ? 'hash ok' : 'HASH MISMATCH'}${result.tier ? ' ' + result.tier : ''}` );
		}
	}
}
