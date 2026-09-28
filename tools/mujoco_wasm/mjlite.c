#include <stdint.h>
#include <string.h>
#include <mujoco/mujoco.h>
#include <emscripten/emscripten.h>

static mjModel* m = 0;
static mjData* d = 0;
static char lastwarn[512];

static void on_warning(const char* msg) { strncpy(lastwarn, msg, sizeof(lastwarn) - 1); }
static void on_error(const char* msg) { strncpy(lastwarn, msg, sizeof(lastwarn) - 1); }

EMSCRIPTEN_KEEPALIVE int mjl_load(const void* buf, int n) {
  mju_user_warning = on_warning;
  mju_user_error = on_error;
  if (d) { mj_deleteData(d); d = 0; }
  if (m) { mj_deleteModel(m); m = 0; }
  m = mj_loadModelBuffer(buf, n);
  if (!m) return 0;
  d = mj_makeData(m);
  return d ? 1 : 0;
}
EMSCRIPTEN_KEEPALIVE int mjl_version(void) { return mj_version(); }
EMSCRIPTEN_KEEPALIVE const char* mjl_lastwarn(void) { return lastwarn; }
EMSCRIPTEN_KEEPALIVE void mjl_reset(void) { mj_resetData(m, d); }
EMSCRIPTEN_KEEPALIVE void mjl_forward(void) { mj_forward(m, d); }
EMSCRIPTEN_KEEPALIVE int mjl_step(int n) {
  int before = d->warning[mjWARN_BADQACC].number + d->warning[mjWARN_BADQVEL].number + d->warning[mjWARN_BADQPOS].number;
  for (int i = 0; i < n; i++) mj_step(m, d);
  return d->warning[mjWARN_BADQACC].number + d->warning[mjWARN_BADQVEL].number + d->warning[mjWARN_BADQPOS].number - before;
}
EMSCRIPTEN_KEEPALIVE int mjl_dim(int k) {
  switch (k) {
    case 0: return m->nq; case 1: return m->nv; case 2: return m->nu; case 3: return m->nsensordata;
    case 4: return m->nsite; case 5: return m->nmocap; case 6: return m->ngeom; case 7: return m->nbody;
    case 8: return d->ncon; case 9: return m->nkey;
  }
  return -1;
}
EMSCRIPTEN_KEEPALIVE double* mjl_ptr(int k) {
  switch (k) {
    case 0: return d->qpos; case 1: return d->qvel; case 2: return d->ctrl; case 3: return d->sensordata;
    case 4: return d->site_xpos; case 5: return d->site_xmat; case 6: return d->mocap_pos; case 7: return d->mocap_quat;
    case 8: return d->xpos; case 9: return d->xquat; case 10: return d->geom_xpos; case 11: return d->qacc_warmstart;
    case 12: return &d->time; case 13: return m->geom_size; case 14: return m->geom_rbound; case 15: return m->geom_aabb;
    case 16: return &m->opt.timestep; case 17: return m->key_qpos; case 18: return m->qpos0; case 19: return d->act;
    case 20: return d->qacc; case 21: return d->actuator_force; case 22: return d->geom_xmat;
  }
  return 0;
}
EMSCRIPTEN_KEEPALIVE int mjl_contact(int k, double* out) {
  if (k < 0 || k >= d->ncon) return 0;
  out[0] = d->contact[k].geom[0]; out[1] = d->contact[k].geom[1]; out[2] = d->contact[k].dist;
  return 1;
}
