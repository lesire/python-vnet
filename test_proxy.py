from filters.filter import create_filter
from proxies.proxy import create_proxy
from Queue import Queue

filters = {None: {None: [create_filter("pass")]}}

queue = Queue()

proxy = create_proxy("queue", filters, queue=queue)

for i in range(5):
    queue.put(str(i))
    proxy.run()

filters[None][None] += [create_filter("block")]
for i in range(5):
    queue.put(str(i))
    proxy.run()

