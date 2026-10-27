// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Heap growth gate: steps a scene of verify/scenes/traj.as under --runtime frame with a frame boundary after
// each step and prints the wasm memory pages after step 20 and step 200. A settled scene must not grow.
//   node verify/heap.mjs <scene> [--wasm=path] [--steps=200]
// Exit code 0 when the pages are equal.

import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { loadWasm, verifyDir } from './lib.mjs';

// traj.mjs runs on import, so read its scene list from the source
const trajSource = readFileSync( join( verifyDir, 'traj.mjs' ), 'utf8' );
const sceneNames = JSON.parse( trajSource.match( /export const sceneNames = (\[[^\]]*\])/ )[1].replace( /'/g, '"' ).replace( /,\s*\]/, ']' ) );

const args = process.argv.slice( 2 );
const flags = Object.fromEntries( args.filter( ( a ) => a.startsWith( '--' ) ).map( ( a ) => a.slice( 2 ).split( '=' ) ) );
const scene = args.find( ( a ) => !a.startsWith( '--' ) ) ?? 'pyramid';
const wasmPath = flags.wasm ?? join( verifyDir, 'scenes', 'out', 'as', 'traj.wasm' );
const steps = Number( flags.steps ?? 200 );

const { exports: e, frameBoundary } = loadWasm( wasmPath );
if ( !e.create( sceneNames.indexOf( scene ), 1 ) )
{
	console.error( `create failed for ${scene}` );
	process.exit( 2 );
}
frameBoundary();

let pages20 = 0;
for ( let i = 0; i < steps; ++i )
{
	e.step( i );
	frameBoundary();
	if ( i + 1 === 20 )
	{
		pages20 = e.memory.buffer.byteLength / 65536;
	}
}
const pagesEnd = e.memory.buffer.byteLength / 65536;
console.log( `${scene}: pages after step 20 ${pages20}, after step ${steps} ${pagesEnd}` );
process.exitCode = pages20 === pagesEnd ? 0 : 1;
