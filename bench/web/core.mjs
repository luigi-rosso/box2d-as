// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Loads one web module (C through emscripten, or the port's rasc bake) and runs a scene, timing each step
// with performance.now and recording its b2Profile fields and state hash. Shared by node and the browser.

// The order of s_sceneNames in verify/scenes/scenes.c
export const SCENES = [ 'pyramid', 'shape_rain', 'chain_car', 'joints', 'dynamic_mover', 'churn', 'bullets',
	'sensors', 'events', 'queries', 'mover', 'many_bodies', 'large_pyramid', 'joint_grid', 'rain', 'smash', 'spinner',
	'tumbler' ];
export const STAGES = [ 'step', 'pairs', 'collide', 'solve', 'solverSetup', 'constraints', 'prepareConstraints',
	'integrateVelocities', 'warmStart', 'solveImpulses', 'integratePositions', 'relaxImpulses', 'restitution',
	'storeImpulses', 'splitIslands', 'transforms', 'sensorHits', 'jointEvents', 'hitEvents', 'refit', 'bullets',
	'sleepIslands', 'sensors' ];

// fetchBytes(name) returns the bytes of an AnimaScript module; importGlue(name) the emscripten factory
export async function load( impl, { fetchBytes, importGlue } )
{
	if ( impl.startsWith( 'c_' ) )
	{
		const factory = await importGlue( impl );
		const m = await factory();
		return {
			create: m._create, step: m._step, stateHash: () => m._stateHash() >>> 0, profileField: m._profileField,
			frameBoundary: () => {},
		};
	}
	const bytes = await fetchBytes( impl );
	const imports = { rive_rt_v1: { now: () => performance.now() } };
	const { instance } = await WebAssembly.instantiate( bytes, imports );
	const e = instance.exports;
	return {
		create: e.create, step: e.step, stateHash: () => e.stateHash() >>> 0, profileField: e.profileField,
		frameBoundary: () => e.frameBoundary(),
	};
}

export function run( m, scene, steps, workers )
{
	const sceneIndex = SCENES.indexOf( scene );
	if ( sceneIndex < 0 || !m.create( sceneIndex, workers ) )
	{
		throw new Error( `create failed for ${scene}` );
	}
	m.frameBoundary();
	const ms = new Array( steps );
	const stages = new Array( steps * STAGES.length );
	const hashes = new Array( steps );
	for ( let i = 0; i < steps; ++i )
	{
		const t0 = performance.now();
		m.step( i );
		ms[i] = performance.now() - t0;
		for ( let k = 0; k < STAGES.length; ++k )
		{
			stages[i * STAGES.length + k] = m.profileField( k );
		}
		hashes[i] = m.stateHash();
		// The frame collector runs between steps, outside the timed step, as in rive
		m.frameBoundary();
	}
	return { ms, stages, hashes };
}

// The per step state hashes of a version 2 trajectory (header in verify/scenes/traj.c): FNV-1a over words 4..10
// of each 11 word body record, p.x p.y q.c q.s v.x v.y w
export function recordingHashes( bytes )
{
	const w = new Uint32Array( bytes.buffer, bytes.byteOffset, bytes.byteLength >> 2 );
	const counters = w[14];
	const hashes = [];
	let i = 16;
	while ( i < w.length )
	{
		const n = w[i + 1];
		i += 2 + counters;
		let h = 2166136261;
		for ( let b = 0; b < n; ++b )
		{
			const r = i + b * 11 + 4;
			for ( let k = 0; k < 7; ++k )
			{
				const bits = w[r + k];
				for ( let s = 0; s < 4; ++s )
				{
					h = Math.imul( h ^ ( ( bits >>> ( 8 * s ) ) & 0xff ), 16777619 ) >>> 0;
				}
			}
		}
		hashes.push( h );
		i += n * 11;
		// probe word count, then the probes
		i += 1 + w[i];
	}
	return hashes;
}

// First step whose hash differs, or -1; steps past the recording are not compared
export function firstMismatch( hashes, reference )
{
	const n = Math.min( hashes.length, reference.length );
	for ( let i = 0; i < n; ++i )
	{
		if ( hashes[i] !== reference[i] )
		{
			return i;
		}
	}
	return -1;
}

function median( sorted )
{
	const n = sorted.length;
	return n & 1 ? sorted[n >> 1] : 0.5 * ( sorted[n / 2 - 1] + sorted[n / 2] );
}

// Steps from warmup on: min and median ms per step, and each stage's mean. mean1 is the mean over steps 1 on,
// what bench/native/bench.c reports.
export function summarize( raw, warmup )
{
	const timed = raw.ms.slice( warmup );
	const sorted = [ ...timed ].sort( ( a, b ) => a - b );
	const stages = {};
	for ( let k = 0; k < STAGES.length; ++k )
	{
		let sum = 0;
		for ( let i = warmup; i < raw.ms.length; ++i )
		{
			sum += raw.stages[i * STAGES.length + k];
		}
		stages[STAGES[k]] = sum / timed.length;
	}
	const rest = raw.ms.slice( 1 );
	return {
		min: sorted[0], median: median( sorted ), mean: timed.reduce( ( a, b ) => a + b, 0 ) / timed.length,
		mean1: rest.reduce( ( a, b ) => a + b, 0 ) / rest.length, stages,
	};
}
