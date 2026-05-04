# Klevebrand All-Drone Controller

Cross-platform C++ drone controller library for all gyroscope/IMU-controlled drones. Runs on embedded microcontrollers and natively on Windows for simulation and testing.

## Coding Principles

### No duplicate state
Never store the same data in two places. Duplicate state creates hard-to-fix inconsistency bugs, increases maintenance cost, and slows development. If two things must agree, one must own the value and the other must derive it.

### Law of Demeter
Never chain more than one call on a returned object. `a.b().c()` violates this principle. Each unit of code should only talk to its immediate collaborators. Following this makes code dramatically easier to use and to refactor.

### Right data structure for the job
If uniqueness must be enforced, use `std::set` or `std::unordered_set` — not `std::vector`. Choose containers whose invariants match the problem. Don't reach for a list when a set is what you actually need.

### Never return nullptr
Return an empty container (`std::vector{}`, `std::optional<T>` without a value, etc.) instead of `nullptr`. Callers should never have to null-check a returned value. Prefer `std::optional<T>` over raw pointers for optional results.

### Prefer immutable classes
Set all fields in the constructor. Avoid setters. Mark member variables `const` where possible. Immutable objects are easier to reason about, test, and use safely across threads. Exception: plain data structs used as configuration or transfer objects may need setters.

### Avoid void methods
Methods should return something meaningful — a result, a status, or `*this` for chaining. Void methods hide outcomes and make call sites harder to verify.

### Decouple models from storage
Models must be constructable from raw data alone, with no dependency on EEPROM, file I/O, or any other storage. Provide a constructor that takes all required values directly. If a convenience constructor that reads from storage is needed, it must delegate to the data constructor — all real construction flows through the storage-free path. This makes unit-testing trivial. No static `load()` methods.

### Transactions and atomicity
If multiple state changes must succeed or fail together, ensure they are atomic. On embedded targets this may mean temporarily disabling interrupts, using a double-buffer, or writing to EEPROM in a single committed block. Never leave state half-updated.

### Exception and error handling
Handle errors at the right boundary. Do not silently swallow failures. Provide enough context in error paths to diagnose the problem. On embedded targets where exceptions are disabled, use explicit return codes or `std::optional`/`std::expected` rather than ignoring failure silently.
