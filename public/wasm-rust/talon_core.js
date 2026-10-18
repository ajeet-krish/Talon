/* @ts-self-types="./talon_core.d.ts" */

/**
 * One steppable mission run.
 */
export class Session {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        SessionFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_session_free(ptr, 0);
    }
    /**
     * Advance up to `steps` plant steps. Returns how many were advanced.
     *
     * This is the frame-loop entry point: the page asks for a chunk of work,
     * gets the count back, and can measure how long the chunk took.
     * @param {number} steps
     * @returns {number}
     */
    advance(steps) {
        const ret = wasm.session_advance(this.__wbg_ptr, steps);
        return ret >>> 0;
    }
    /**
     * Live altitude above the ground plane (m, positive up).
     *
     * Ground is the scenario start height; height is the negated NED down
     * coordinate, which is the spine the scene already renders on.
     * @returns {number}
     */
    altitude_m() {
        const ret = wasm.session_altitude_m(this.__wbg_ptr);
        return ret;
    }
    /**
     * Battery state of charge in [0, 1].
     * @returns {number}
     */
    get battery_soc() {
        const ret = wasm.session_battery_soc(this.__wbg_ptr);
        return ret;
    }
    /**
     * The schema 1.1 flight document for the run so far, as JSON.
     *
     * `generated_utc` is supplied by the caller because this crate has no clock.
     * @param {string} generated_utc
     * @returns {string}
     */
    document_json(generated_utc) {
        let deferred3_0;
        let deferred3_1;
        try {
            const ptr0 = passStringToWasm0(generated_utc, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            const len0 = WASM_VECTOR_LEN;
            const ret = wasm.session_document_json(this.__wbg_ptr, ptr0, len0);
            var ptr2 = ret[0];
            var len2 = ret[1];
            if (ret[3]) {
                ptr2 = 0; len2 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred3_0 = ptr2;
            deferred3_1 = len2;
            return getStringFromWasm0(ptr2, len2);
        } finally {
            wasm.__wbindgen_free(deferred3_0, deferred3_1, 1);
        }
    }
    /**
     * Configured run duration (s).
     * @returns {number}
     */
    get duration_s() {
        const ret = wasm.session_duration_s(this.__wbg_ptr);
        return ret;
    }
    /**
     * Live estimated position, NED (m).
     * @returns {Float64Array}
     */
    ekf_position() {
        const ret = wasm.session_ekf_position(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Whether the run has reached its configured duration.
     * @returns {boolean}
     */
    get finished() {
        const ret = wasm.session_finished(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Create a session from a scenario JSON document.
     *
     * The JSON is the same shape served at `public/data/live/configs.json`. If
     * the scenario asks for `auto_arm`, arm and takeoff are queued now so the
     * first control tick starts the mission, matching the batch harness.
     * @param {string} config_json
     */
    constructor(config_json) {
        const ptr0 = passStringToWasm0(config_json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.session_new(ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        this.__wbg_ptr = ret[0];
        SessionFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Current flight-phase label, lowercase.
     * @returns {string}
     */
    phase() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.session_phase(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * Convene the estimator at rest, on a negative clock, before the timed run.
     *
     * Idempotent, so a caller that prepares twice does not pay twice or shift
     * the clock. Must be called before the first step for the estimate to be
     * converged when the mission starts.
     */
    prepare() {
        wasm.session_prepare(this.__wbg_ptr);
    }
    /**
     * Whether the convergence pre-roll has run.
     * @returns {boolean}
     */
    get prepared() {
        const ret = wasm.session_prepared(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Fraction of the run completed, in [0, 1].
     * @returns {number}
     */
    get progress() {
        const ret = wasm.session_progress(this.__wbg_ptr);
        return ret;
    }
    /**
     * Advance to the end of the run. Returns the number of steps advanced.
     * @returns {number}
     */
    run_to_end() {
        const ret = wasm.session_run_to_end(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Scenario id, written to `meta.scenario`.
     * @returns {string}
     */
    get scenario_id() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.session_scenario_id(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * Advance one plant step. Returns true once the run is finished.
     * @returns {boolean}
     */
    step() {
        const ret = wasm.session_step(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Plant steps advanced so far.
     * @returns {number}
     */
    get steps_done() {
        const ret = wasm.session_steps_done(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Headline run numbers, as JSON.
     * @returns {string}
     */
    summary_json() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.session_summary_json(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * Sim time (s). Runs from 0 to `duration_s`; negative during the pre-roll.
     * @returns {number}
     */
    get time() {
        const ret = wasm.session_time(this.__wbg_ptr);
        return ret;
    }
    /**
     * Plant steps in the whole run.
     * @returns {number}
     */
    get total_steps() {
        const ret = wasm.session_total_steps(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Live truth position, NED (m).
     * @returns {Float64Array}
     */
    truth_position() {
        const ret = wasm.session_truth_position(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Live truth velocity, NED (m/s).
     * @returns {Float64Array}
     */
    truth_velocity() {
        const ret = wasm.session_truth_velocity(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
}
if (Symbol.dispose) Session.prototype[Symbol.dispose] = Session.prototype.free;

/**
 * Core version, for the UI to display alongside the run.
 * @returns {string}
 */
export function core_version() {
    let deferred1_0;
    let deferred1_1;
    try {
        const ret = wasm.core_version();
        deferred1_0 = ret[0];
        deferred1_1 = ret[1];
        return getStringFromWasm0(ret[0], ret[1]);
    } finally {
        wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
    }
}

/**
 * Run a scenario to completion and return its flight document.
 *
 * The one-call form of [`Session`], for tests and for any caller that does not
 * need to step the run itself.
 * @param {string} config_json
 * @param {string} generated_utc
 * @returns {string}
 */
export function run_scenario(config_json, generated_utc) {
    let deferred4_0;
    let deferred4_1;
    try {
        const ptr0 = passStringToWasm0(config_json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passStringToWasm0(generated_utc, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.run_scenario(ptr0, len0, ptr1, len1);
        var ptr3 = ret[0];
        var len3 = ret[1];
        if (ret[3]) {
            ptr3 = 0; len3 = 0;
            throw takeFromExternrefTable0(ret[2]);
        }
        deferred4_0 = ptr3;
        deferred4_1 = len3;
        return getStringFromWasm0(ptr3, len3);
    } finally {
        wasm.__wbindgen_free(deferred4_0, deferred4_1, 1);
    }
}
function __wbg_get_imports() {
    const import0 = {
        __proto__: null,
        __wbg___wbindgen_throw_41e9ee4f547fc59a: function(arg0, arg1) {
            throw new Error(getStringFromWasm0(arg0, arg1));
        },
        __wbindgen_generic_0000000000000001: function(arg0, arg1) {
            // Cast intrinsic for `Ref(String) -> Externref`.
            const ret = getStringFromWasm0(arg0, arg1);
            return ret;
        },
        __wbindgen_init_externref_table: function() {
            const table = wasm.__wbindgen_externrefs;
            const offset = table.grow(4);
            table.set(0, undefined);
            table.set(offset + 0, undefined);
            table.set(offset + 1, null);
            table.set(offset + 2, true);
            table.set(offset + 3, false);
        },
    };
    return {
        __proto__: null,
        "./talon_core_bg.js": import0,
    };
}

const SessionFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_session_free(ptr, 1));

function getArrayF64FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat64ArrayMemory0().subarray(ptr / 8, ptr / 8 + len);
}

let cachedFloat64ArrayMemory0 = null;
function getFloat64ArrayMemory0() {
    if (cachedFloat64ArrayMemory0 === null || cachedFloat64ArrayMemory0.byteLength === 0) {
        cachedFloat64ArrayMemory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    return decodeText(ptr >>> 0, len);
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function passStringToWasm0(arg, malloc, realloc) {
    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }
    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = cachedTextEncoder.encodeInto(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

function takeFromExternrefTable0(idx) {
    const value = wasm.__wbindgen_externrefs.get(idx);
    wasm.__externref_table_dealloc(idx);
    return value;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

const cachedTextEncoder = new TextEncoder();

if (!('encodeInto' in cachedTextEncoder)) {
    cachedTextEncoder.encodeInto = function (arg, view) {
        const buf = cachedTextEncoder.encode(arg);
        view.set(buf);
        return {
            read: arg.length,
            written: buf.length
        };
    };
}

let WASM_VECTOR_LEN = 0;

let wasmModule, wasmInstance, wasm;
function __wbg_finalize_init(instance, module) {
    wasmInstance = instance;
    wasm = instance.exports;
    wasmModule = module;
    cachedFloat64ArrayMemory0 = null;
    cachedUint8ArrayMemory0 = null;
    wasm.__wbindgen_start();
    return wasm;
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (!module.ok) {
            throw new Error(`failed to fetch Wasm: ${module.status} ${module.statusText} fetching '${module.url}'`);
        }

        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = expectedResponseType(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else { throw e; }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }

    function expectedResponseType(type) {
        switch (type) {
            case 'basic': case 'cors': case 'default': return true;
        }
        return false;
    }
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (module !== undefined) {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (module_or_path !== undefined) {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (module_or_path === undefined) {
        module_or_path = new URL('talon_core_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync, __wbg_init as default };
