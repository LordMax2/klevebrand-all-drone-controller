#include "base_pid_repository.h"

PidConstants_t BasePidRepository::get() { return {0, 0, 0, 0, 0, 0, 0, 0, 0}; }

PidConstants_t BasePidRepository::get(int key) { return {0, 0, 0, 0, 0, 0, 0, 0, 0}; }

void BasePidRepository::setup() {
}

void BasePidRepository::save(PidConstants_t &pid_constants) {
}

void BasePidRepository::save(int key, PidConstants_t &pid_constants) {
}
