// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reads trajectory files written by traj.c (format in traj.c).
//   node dump.mjs print <file> [--step=N] [--body=I] [--probes]
//   node dump.mjs diff <a> <b> [--all]
// diff compares bodies, the physics counters and the probe words; --all also compares the build and
// worker dependent counters. Exit code 0 when identical, 1 when they differ. print shows the probe
// words of each step with --probes, as integers and floats.

import { readFileSync } from 'node:fs';

export const counterNames = [
	'bodyCount', 'shapeCount', 'contactCount', 'jointCount', 'islandCount', 'awakeBodyCount',
	'awakeContactCount', 'recycledContactCount', 'staticTreeHeight', 'treeHeight', 'contactBeginCount',
	'contactEndCount', 'contactHitCount', 'sensorBeginCount', 'sensorEndCount', 'bodyMoveCount', 'jointEventCount',
	...Array.from( { length: 24 }, ( _, i ) => `colorCounts[${i}]` ),
	'taskCount', 'stackUsed', 'byteCountLo', 'byteCountHi', 'workerCount', 'reserved',
];
export const envCounterStart = 41;

export const bodyFields = [ 'index', 'flags', 'id.index1', 'id.generation', 'p.x', 'p.y', 'q.c', 'q.s', 'v.x', 'v.y', 'w' ];
const bodyFloatStart = 4;

export function readTrajectory( path )
{
	const buffer = readFileSync( path );
	const u32 = new Uint32Array( buffer.buffer, buffer.byteOffset, buffer.byteLength >> 2 );
	const f32 = new Float32Array( buffer.buffer, buffer.byteOffset, buffer.byteLength >> 2 );
	if ( u32[0] !== 0x52543242 || ( u32[1] !== 1 && u32[1] !== 2 ) )
	{
		throw new Error( `${path}: not a version 1 or 2 trajectory` );
	}
	const version = u32[1];
	const nameBytes = buffer.subarray( 8, 40 );
	const end = nameBytes.indexOf( 0 );
	const name = nameBytes.subarray( 0, end < 0 ? 32 : end ).toString();
	const header = {
		name,
		version,
		stepCount: u32[10],
		workerCount: u32[11],
		timeStep: f32[12],
		subStepCount: u32[13],
		counterCount: u32[14],
	};

	const steps = [];
	let o = 16;
	while ( o < u32.length )
	{
		const stepIndex = u32[o];
		const bodyCount = u32[o + 1];
		const counterOffset = o + 2;
		const bodyOffset = counterOffset + header.counterCount;
		o = bodyOffset + bodyCount * bodyFields.length;
		let probeCount = 0;
		let probeOffset = o;
		if ( version >= 2 )
		{
			probeCount = u32[o];
			probeOffset = o + 1;
			o = probeOffset + probeCount;
		}
		steps.push( { stepIndex, bodyCount, counterOffset, bodyOffset, probeCount, probeOffset } );
	}
	return { path, header, steps, u32, f32 };
}

const hex = ( x ) => x.toString( 16 ).padStart( 8, '0' );

function formatBody( t, s, b )
{
	const base = s.bodyOffset + b * bodyFields.length;
	const u = t.u32;
	const f = t.f32;
	const flags = u[base + 1];
	const vec = ( k, n ) => Array.from( { length: n }, ( _, i ) => f[base + k + i].toPrecision( 9 ) ).join( ' ' );
	return `body ${u[base]} id ${u[base + 2]}.${u[base + 3]} valid ${flags & 1} awake ${( flags >> 1 ) & 1} type ${( flags >> 4 ) & 15}` +
		` p [${vec( 4, 2 )}] q [${vec( 6, 2 )}] v [${vec( 8, 2 )}] w ${vec( 10, 1 )}`;
}

// A probe word as an integer, and as a float when its bits look like one
function probeText( t, index )
{
	const u = t.u32[index];
	if ( u === 0x80000000 )
	{
		return '-0';
	}
	const exponent = ( u >>> 23 ) & 255;
	if ( u === 0 || exponent === 0 || exponent === 255 || ( u | 0 ) > -0x10000 && ( u | 0 ) < 0x10000 )
	{
		return `${u | 0}`;
	}
	return `${t.f32[index].toPrecision( 7 )}`;
}

function formatProbes( t, s, first, count )
{
	const words = [];
	for ( let k = first; k < Math.min( s.probeCount, first + count ); ++k )
	{
		words.push( probeText( t, s.probeOffset + k ) );
	}
	return words.join( ' ' );
}

function print( t, stepFilter, bodyFilter, probes )
{
	const h = t.header;
	console.log( `${h.name}: version ${h.version} steps ${h.stepCount} workers ${h.workerCount} dt ${h.timeStep}` +
		` substeps ${h.subStepCount} counters ${h.counterCount}` );
	for ( const s of t.steps )
	{
		if ( stepFilter !== undefined && s.stepIndex !== stepFilter )
		{
			continue;
		}
		const c = counterNames.slice( 0, 17 ).map( ( n, i ) => `${n} ${t.u32[s.counterOffset + i] | 0}` ).join( ', ' );
		console.log( `step ${s.stepIndex} bodies ${s.bodyCount} probes ${s.probeCount}: ${c}` );
		if ( probes && s.probeCount > 0 )
		{
			console.log( `  probes: ${formatProbes( t, s, 0, s.probeCount )}` );
		}
		for ( let b = 0; b < s.bodyCount; ++b )
		{
			if ( bodyFilter === undefined || bodyFilter === b )
			{
				console.log( '  ' + formatBody( t, s, b ) );
			}
		}
	}
}

function valueText( t, index, isFloat )
{
	return isFloat ? `${t.f32[index].toPrecision( 9 )} (${hex( t.u32[index] )})` : `${t.u32[index] | 0}`;
}

// Returns a summary; prints the first differing body field and counter
export function diff( a, b, all, quiet )
{
	const result = { identical: true, firstBody: null, firstCounter: null, firstProbe: null, differingSteps: 0 };
	const report = ( ...args ) => quiet || console.log( ...args );
	if ( a.header.counterCount !== b.header.counterCount || a.header.version !== b.header.version )
	{
		report( 'layouts differ' );
		result.identical = false;
		return result;
	}
	const counterLimit = all ? a.header.counterCount : envCounterStart;
	const stepCount = Math.min( a.steps.length, b.steps.length );
	if ( a.steps.length !== b.steps.length )
	{
		report( `step counts differ: ${a.steps.length} vs ${b.steps.length}` );
		result.identical = false;
	}
	for ( let si = 0; si < stepCount; ++si )
	{
		const sa = a.steps[si];
		const sb = b.steps[si];
		let stepDiffers = false;

		if ( sa.bodyCount !== sb.bodyCount )
		{
			stepDiffers = true;
			if ( result.firstBody === null )
			{
				result.firstBody = { step: sa.stepIndex, body: -1, field: 'bodyCount' };
				report( `first body difference: step ${sa.stepIndex} body count ${sa.bodyCount} vs ${sb.bodyCount}` );
			}
		}

		for ( let i = 0; i < counterLimit; ++i )
		{
			const va = a.u32[sa.counterOffset + i];
			const vb = b.u32[sb.counterOffset + i];
			if ( va !== vb )
			{
				stepDiffers = true;
				if ( result.firstCounter === null )
				{
					result.firstCounter = { step: sa.stepIndex, counter: counterNames[i], index: i };
					report( `first counter difference: step ${sa.stepIndex} ${counterNames[i]} ${va | 0} vs ${vb | 0}` );
				}
				break;
			}
		}

		const bodyCount = Math.min( sa.bodyCount, sb.bodyCount );
		const words = bodyCount * bodyFields.length;
		for ( let k = 0; k < words; ++k )
		{
			if ( a.u32[sa.bodyOffset + k] !== b.u32[sb.bodyOffset + k] )
			{
				stepDiffers = true;
				if ( result.firstBody === null )
				{
					const body = Math.floor( k / bodyFields.length );
					const field = k % bodyFields.length;
					const isFloat = field >= bodyFloatStart;
					result.firstBody = { step: sa.stepIndex, body, field: bodyFields[field] };
					report( `first body difference: step ${sa.stepIndex} body ${body} ${bodyFields[field]} ` +
						`${valueText( a, sa.bodyOffset + k, isFloat )} vs ${valueText( b, sb.bodyOffset + k, isFloat )}` );
					report( '  a ' + formatBody( a, sa, body ) );
					report( '  b ' + formatBody( b, sb, body ) );
				}
				break;
			}
		}

		const probeLimit = Math.min( sa.probeCount, sb.probeCount );
		let probeIndex = sa.probeCount === sb.probeCount ? -1 : probeLimit;
		for ( let k = 0; k < probeLimit; ++k )
		{
			if ( a.u32[sa.probeOffset + k] !== b.u32[sb.probeOffset + k] )
			{
				probeIndex = k;
				break;
			}
		}
		if ( probeIndex >= 0 )
		{
			stepDiffers = true;
			if ( result.firstProbe === null )
			{
				const va = probeIndex < sa.probeCount ? probeText( a, sa.probeOffset + probeIndex ) : '-';
				const vb = probeIndex < sb.probeCount ? probeText( b, sb.probeOffset + probeIndex ) : '-';
				result.firstProbe = { step: sa.stepIndex, index: probeIndex, a: va, b: vb };
				const from = Math.max( 0, probeIndex - 4 );
				report( `first probe difference: step ${sa.stepIndex} word ${probeIndex} of ${sa.probeCount} vs ${sb.probeCount}: ` +
					`${va} vs ${vb}` );
				report( `  a [${from}..] ${formatProbes( a, sa, from, 12 )}` );
				report( `  b [${from}..] ${formatProbes( b, sb, from, 12 )}` );
			}
		}

		if ( stepDiffers )
		{
			result.identical = false;
			result.differingSteps += 1;
		}
	}

	if ( result.identical )
	{
		report( `identical over ${stepCount} steps${all ? '' : ' (environment counters ignored)'}` );
	}
	else
	{
		report( `${result.differingSteps} of ${stepCount} steps differ` );
	}
	return result;
}

function main()
{
	const args = process.argv.slice( 2 );
	const flags = Object.fromEntries( args.filter( ( a ) => a.startsWith( '--' ) ).map( ( a ) =>
	{
		const [ k, v ] = a.slice( 2 ).split( '=' );
		return [ k, v === undefined ? true : v ];
	} ) );
	const positional = args.filter( ( a ) => !a.startsWith( '--' ) );
	const command = positional[0];

	if ( command === 'print' && positional.length === 2 )
	{
		const step = flags.step === undefined ? undefined : Number( flags.step );
		const body = flags.body === undefined ? undefined : Number( flags.body );
		print( readTrajectory( positional[1] ), step, body, flags.probes === true );
		return 0;
	}

	if ( command === 'diff' && positional.length === 3 )
	{
		const result = diff( readTrajectory( positional[1] ), readTrajectory( positional[2] ), flags.all === true, false );
		return result.identical ? 0 : 1;
	}

	console.error( 'usage: node dump.mjs print <file> [--step=N] [--body=I] [--probes]\n       node dump.mjs diff <a> <b> [--all]' );
	return 2;
}

if ( import.meta.url === `file://${process.argv[1]}` )
{
	process.exitCode = main();
}
