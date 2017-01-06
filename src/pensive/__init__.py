'''

# Pensive

A hierarchical key-value database for Python objects.

## Design

The database provides a hierarchical key-value store much like the built-in
Python dictionary. The key is a forward-slash (`/`) delimited path much like
a UNIX file path or URL. The value can be any Python object that can be serialized
in JavaScript Object Notation (JSON).

The database supports three primary operations.
 - `get`: retrieve a value
 - `put`: store a value
 - `delete`: remove a value
Each operation is valid for any key. Any key which has not been set is implicitly
treated a holding the value `None`. Thus, `get` on a non-existing key will return
`None` without raising an error. Similarly, `put` will create as many subkeys and
overwrite existing values as needed to store the requested value at that key. A
consequent of this is that `delete` is equivalent to a `put` with a `None` value.
(In fact, this is how `delete` is implemented.) If a `get` is performed on a key
that contains subkeys, a Python dictionary of all subkeys and values below that key
is returned.

The database supports have multiple instances or `Store`s that share value between
them. A default instance always exists. At any time, a new instance can be created.
The new instance can be empty or a copy of an existing instance without duplicating
values. This later option is advantageous when the instance contains large values
such as base-64 encoded images.

### Example

Suppose the database starts empty.
```
{}
```
After putting the value `4` at key `a/b/c`, the database creates the
necessary structure to contain the nested keys `a`, `b`, and `c`.
```
{
    'a': {
        'b': {
            'c': 4
        }
    }
}
The `get` operations below will have the following results.
 - `get('a/b/c') -> 4`
 - `get(`a`) -> {'b': {'c': 4}}`
 - `get('random') -> None`

```
After putting the value `{'s': 2}` at key `a/d`:
```
{
    'a': {
        'b': {
            'c': 4
        },
        'd': {
            's': 2
        }
    }
}
```
The `get` operations below will have the following results.
 - `get('a/b/c') -> 4`
 - `get('a/d/s') -> 2
 - `get(`a`) -> {'b': {'c': 4}, 'd': {'s': 2}}`

## Usage

The following code snippet starts a database server on all interfaces
at the default port.
```
from pensive.server import PensiveServer

server = PensiveServer()
server.run()
```
The `PensiveServer.run` method blocks until a `KeyboardInterrupt`
is raised. The address and port to which to bind can be specified
as arguments to the `PensiveServer` constructor.

The following code snippet creates a database client that connects
to `localhost` on the default port.
```
from pensive.client import PensiveClient

client = PensiveClient()
# access the default store
store = client.default()
# access an existing named instance
store = client.store('existing')
# create a new instance
store = client.create('new')

store.put('a/b/c', 4)
store.get('a/b/c') # -> 4
```
Referer to the `PensiveClient` and `StoreProxy` classes for available
database operations and flags.
'''

DEFAULT_PORT = 8888
