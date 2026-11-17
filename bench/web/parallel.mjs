// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The threaded step in a browser: bakes bench/as with the step on parallel calls and HASHLOG, runs each bake on a
// cross origin isolated page over librive's threads build at each part count, and holds every step to the native
// recording and to that part count through verify/threads.mjs, as verify/threads.sh does under rive-cli.
//   PLAYWRIGHT=<dir holding node_modules/playwright-core> HARNESS=<rive>/packages/runtime/tests/web_scripting/out/wasm_threads_release \
//     node bench/web/parallel.mjs [scene...]
// PARTS="1 2 4 8 default": 1 runs every call as one part, n sets the page's part count, default leaves it to librive.
// BROWSER=chromium (Chrome), firefox or webkit. A run gets TIMEOUT seconds, 900 by default. RIVE names rive-cli,
// verify/toolchain.sh's by default; it must be a dev build, which --unsigned-riv needs.

import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { copyFileSync, mkdirSync, mkdtempSync, writeFileSync } from 'node:fs';
import { execFileSync, spawnSync } from 'node:child_process';
import { tmpdir } from 'node:os';
import { dirname, extname, join, normalize } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { SCENES } from './core.mjs';

const here = dirname( fileURLToPath( import.meta.url ) );
const root = join( here, '..', '..' );
const project = join( root, 'bench', 'as' );
const rive = process.env.RIVE ??
	execFileSync( 'bash', [ '-c', `source "${root}/verify/toolchain.sh" && echo "$RIVE"` ], { encoding: 'utf8' } ).trim();
const parts = ( process.env.PARTS ?? '1 2 4 8 default' ).split( /\s+/ ).filter( Boolean );
const named = process.argv.slice( 2 );
const scenes = named.length > 0 ? named : [ 'pyramid', 'joints', 'churn', 'bullets', 'sensors', 'events', 'large_pyramid', 'rain' ];
const timeout = Number( process.env.TIMEOUT ?? 900 ) * 1000;
const out = join( here, 'out', 'parallel' );
mkdirSync( out, { recursive: true } );

// The repo root, with /harness/ mapped to librive's build and the headers shared memory needs
const types = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript', '.wasm': 'application/wasm' };
const server = createServer( async ( req, res ) =>
{
	const pathname = decodeURIComponent( new URL( req.url, 'http://x' ).pathname );
	const path = normalize( pathname.startsWith( '/harness/' ) ? join( process.env.HARNESS, pathname.slice( 9 ) )
		: join( root, pathname ) );
	try
	{
		const body = await readFile( path );
		res.writeHead( 200, { 'Content-Type': types[extname( path )] ?? 'application/octet-stream',
			'Cross-Origin-Opener-Policy': 'same-origin', 'Cross-Origin-Embedder-Policy': 'require-corp',
			'Cache-Control': 'no-store' } );
		res.end( body );
	}
	catch
	{
		res.writeHead( 404 );
		res.end();
	}
} );
await new Promise( ( resolve ) => server.listen( 0, '127.0.0.1', resolve ) );
const origin = `http://127.0.0.1:${server.address().port}`;

const browserName = process.env.BROWSER ?? 'chromium';
const pw = await import( pathToFileURL( join( process.env.PLAYWRIGHT, 'node_modules', 'playwright-core', 'index.mjs' ) ) );
const browser = await pw[browserName].launch( browserName === 'chromium' ? { channel: 'chrome', headless: true }
	: { headless: true } );
console.log( `${browserName} ${browser.version()}` );

const logs = mkdtempSync( join( tmpdir(), 'box2d_parallel_' ) );
let failed = 0;
try
{
	execFileSync( join( project, 'sync.sh' ) );
	for ( const scene of scenes )
	{
		// The scene runs the trajectory gate's step count, bench/as's default
		writeFileSync( join( project, 'options.txt' ), `SCENE=${SCENES.indexOf( scene )} PARALLEL=15 HASHLOG=1\n` );
		execFileSync( rive, [ project, '--unsigned-riv', '--release' ], { stdio: 'ignore' } );
		copyFileSync( join( project, 'build', 'box2d_bench.riv' ), join( out, `${scene}.riv` ) );

		for ( const p of parts )
		{
			const options = p === '1' ? { parallelSafe: true } : p === 'default' ? {} : { parts: Number( p ) };
			const tab = await browser.newPage();
			let lines;
			try
			{
				await tab.goto( `${origin}/bench/web/parallel.html` );
				await tab.waitForFunction( () => window.ready === true );
				// A frame that never returns never reaches the page's own deadline, and evaluate has none
				const evaluated = tab.evaluate( ( [ n, s, o ] ) => window.runRiv( n, s, o ), [ scene, timeout / 2000, options ] );
				evaluated.catch( () => {} );
				const run = await Promise.race( [ evaluated,
					new Promise( ( settle ) => setTimeout( settle, timeout, { error: `timed out after ${timeout / 1000} s`, lines: [] } ) ) ] );
				lines = run.error ? [ `failed ${run.error}`, ...run.lines ]
					: run.crossOriginIsolated ? run.lines : [ 'failed: the page is not cross origin isolated', ...run.lines ];
			}
			catch ( err )
			{
				lines = [ `failed ${String( err ).slice( 0, 200 )}` ];
			}
			finally
			{
				// A page stuck in a frame may not close either; the browser's close ends it
				await Promise.race( [ tab.close().catch( () => {} ), new Promise( ( settle ) => setTimeout( settle, 5000 ) ) ] );
			}
			const log = join( logs, `${scene}_${p}.log` );
			writeFileSync( log, lines.join( '\n' ) + '\n' );
			const check = spawnSync( 'node', [ join( root, 'verify', 'threads.mjs' ),
				join( root, 'verify', 'scenes', 'out', `${scene}_scalar_w1.bin` ), log, `${scene} P${p} ${browserName}`, p ],
			{ encoding: 'utf8' } );
			const size = lines.join( '\n' ).match( /BENCH parallel size=(\d+)/ )?.[1] ?? '-';
			console.log( `${check.stdout.trim()} size ${size}` );
			failed += check.status === 0 ? 0 : 1;
		}
	}
}
finally
{
	writeFileSync( join( project, 'options.txt' ), '' );
	await browser.close();
	server.close();
}
console.log( `passed: ${scenes.length * parts.length - failed}  failed: ${failed}` );
process.exit( failed === 0 ? 0 : 1 );
