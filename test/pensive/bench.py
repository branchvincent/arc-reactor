from multiprocessing import Process
from time import time, sleep

n_clients = 4
n_operations = 100

def server_entry():
    from pensive.server import PensiveServer
    PensiveServer().run()

def client_entry():
    from random import choice, randint, uniform
    from string import ascii_lowercase

    from pensive.client import PensiveClient

    store = PensiveClient().default()
    for i in range(n_operations):
        s = '/'.join([''.join([choice(ascii_lowercase) for i in range(randint(1,10))]) for j in range(randint(1,4))])

        if uniform(0, 1) > 0.5:
            store.get()
        else:
            store.put(s, uniform(0, 1))

server = Process(target=server_entry)
server.start()

print 'waiting for server start'
sleep(1)

clients = [ Process(target=client_entry) for i in range(n_clients) ]

start_time = time()

print 'spawn {} workers with {} operations each'.format(len(clients), n_operations)

for p in clients:
    p.start()

for p in clients:
    p.join()


end_time = time()
diff_time = end_time - start_time

server.terminate()

print '{} ops'.format(len(clients) * n_operations)
print '{:.3f} s'.format(diff_time)
print '{:.1f} op/s'.format(len(clients) * n_operations / diff_time)
print '{:.3f} ms/op'.format(diff_time * 1e3 / len(clients) / n_operations)
