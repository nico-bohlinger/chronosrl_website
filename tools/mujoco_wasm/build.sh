#!/bin/bash
# Usage: tools/mujoco_wasm/build.sh <mujoco checkout with the 3.7.0 tag> <emsdk folder>
set -euo pipefail
MUJOCO=$(cd "$1" && pwd); EMSDK=$(cd "$2" && pwd)
HERE=$(cd "$(dirname "$0")" && pwd); SITE=$(cd "$HERE/../.." && pwd)
WORK=$(mktemp -d); trap 'rm -rf "$WORK"' EXIT
source "$EMSDK/emsdk_env.sh" >/dev/null 2>&1
git -C "$MUJOCO" archive 3.7.0 src include | tar -x -C "$WORK"
git clone -q https://github.com/danfis/libccd.git "$WORK/libccd" && git -C "$WORK/libccd" checkout -q 7931e764a19ef6b21b443376c699bbc9c6d4fba8
mkdir -p "$WORK/ccdcfg/ccd" "$WORK/obj"
printf '#ifndef __CCD_CONFIG_H__\n#define __CCD_CONFIG_H__\n#define CCD_DOUBLE\n#endif\n' > "$WORK/ccdcfg/ccd/config.h"
cd "$WORK"
DEFS="-DMJ_STATIC -D_GNU_SOURCE -DCCD_STATIC_DEFINE -DMUJOCO_DLL_EXPORTS"
INC="-Iinclude -Isrc -Ilibccd/src -Iccdcfg"
OPT="-O3 -fno-fast-math -ffp-contract=off"
objs=()
for f in src/engine/*.c libccd/src/{ccd,mpr,polytope,support,vec3}.c "$HERE/mjlite.c"; do
  o=obj/$(basename "$f").o; emcc -std=c11 $OPT $DEFS $INC -c "$f" -o "$o" & objs+=("$o")
done
for f in src/engine/*.cc src/thread/*.cc; do
  o=obj/$(basename "$f").o; em++ -std=c++20 $OPT $DEFS $INC -fno-exceptions -c "$f" -o "$o" & objs+=("$o")
done
wait
EXP='["_mjl_load","_mjl_version","_mjl_lastwarn","_mjl_reset","_mjl_forward","_mjl_step","_mjl_dim","_mjl_ptr","_mjl_contact","_malloc","_free"]'
em++ $OPT "${objs[@]}" -o mujoco370.mjs \
  -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=loadMuJoCo -sENVIRONMENT=web,worker,node \
  -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=33554432 -sSTACK_SIZE=1048576 -sFILESYSTEM=0 \
  -sEXPORTED_FUNCTIONS="$EXP" -sEXPORTED_RUNTIME_METHODS='["HEAPF64","HEAPU8","HEAP32","UTF8ToString"]' \
  -sASSERTIONS=0 -fno-exceptions
mkdir -p "$SITE/vendor/mujoco"
cp mujoco370.mjs "$SITE/vendor/mujoco/mujoco370.js"
gzip -9 -n -c mujoco370.wasm > "$SITE/vendor/mujoco/mujoco370.wasm.gz"
cp "$MUJOCO/LICENSE" "$SITE/vendor/mujoco/LICENSE"; cp libccd/BSD-LICENSE "$SITE/vendor/mujoco/LICENSE-libccd"
