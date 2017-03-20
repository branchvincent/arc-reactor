# pylint: disable=line-too-long,missing-docstring,invalid-name,protected-access

from .helper import DatabaseDependentTestCase, FakeHTTPClient

from pensive.server import PensiveServer, Store
from pensive.client import StoreProxy, StoreTransaction, PensiveClient

from pensive.coders import register_numpy

class Skip(object):  # pylint: disable=too-few-public-methods
    class ClientTest(DatabaseDependentTestCase):
        def setUp(self):
            super(Skip.ClientTest, self).setUp()
            self.proxy = StoreProxy(self.get_url(''), instance=self.instance, client=FakeHTTPClient(self))

            self.server.stores.setdefault(self.instance, Store()).put(value={'a': 4, 'b': {'c': 2}})

        def test_client_get(self):
            self.assertEqual(self.proxy.get('a'), 4)
            self.assertEqual(self.proxy.get('b/c'), 2)
            self.assertDictEqual(self.proxy.get('b'), {'c': 2})
            self.assertDictEqual(self.proxy.get(), {'a': 4, 'b': {'c': 2}})
            self.assertIsNone(self.proxy.get('nonexistent'))

            with self.assertRaises(KeyError):
                self.proxy.get('nonexistent', strict=True)
            self.assertEqual(self.proxy.get('nonexistent', default=1234), 1234)
            #with self.assertRaises(RuntimeError):
            #    self.proxy.get('a', badoption=True)

        def test_client_mutil_get(self):
            self.assertDictEqual(self.proxy.multi_get(['a', 'b']), {'a': 4, 'b': {'c': 2}})
            self.assertDictEqual(self.proxy.multi_get(['c'], root='b'), {'c': 2})
            with self.assertRaises(RuntimeError):
                self.proxy.multi_get([])

        def test_client_put(self):
            self.proxy.put('b', 5)
            self.assertDictEqual(self.server.stores[self.instance].get(), {'a': 4, 'b': 5})
            self.proxy.put('c', {'d': 6})
            self.assertDictEqual(self.server.stores[self.instance].get(), {'a': 4, 'b': 5, 'c': {'d': 6}})

            with self.assertRaises(KeyError):
                self.proxy.put('c', 5, strict=True)
            #with self.assertRaises(RuntimeError):
            #    self.proxy.put('c', 5, badoption=True)

        def test_client_multi_put(self):
            self.proxy.multi_put({'b': 5, 'c': {'d': 6}})
            self.assertDictEqual(self.server.stores[self.instance].get(), {'a': 4, 'b': 5, 'c': {'d': 6}})

        def test_client_multi_put_root(self):
            self.proxy.multi_put({'d': 7}, root='b')
            self.assertDictEqual(self.server.stores[self.instance].get(), {'a': 4, 'b': {'c': 2, 'd': 7}})

        def test_client_delete(self):
            self.proxy.delete('a')
            self.assertDictContainsSubset(self.server.stores[self.instance].get(), {'b': {'c': 2}})
            #with self.assertRaises(RuntimeError):
            #    self.proxy.delete('adsfads', badoption=True)

        def test_client_delete_nested(self):
            self.proxy.delete('b/c')
            self.assertDictEqual(self.server.stores[self.instance].get(), {'a': 4})

        def test_client_multi_delete(self):
            self.proxy.multi_delete(['a', 'b/c'])
            self.assertIsNone(self.server.stores[self.instance].get())

        def test_client_multi_delete_root(self):
            self.proxy.multi_delete(['c'], root='b')
            self.assertDictEqual(self.server.stores[self.instance].get(), {'a': 4})

class ClientTest_Default(Skip.ClientTest):
    instance = None

class ClientTest_Instance(Skip.ClientTest):
    instance = 'random'

class ClientTest_Transaction(DatabaseDependentTestCase):
    def setUp(self):
        super(ClientTest_Transaction, self).setUp()
        self.proxy = StoreProxy(self.get_url(''), client=FakeHTTPClient(self))
        self.trans = StoreTransaction(self.proxy)

        self.server.stores[None].put(value={'a': 4, 'b': {'c': 2}})

    def test_transaction_get(self):
        self.assertEqual(self.trans.get('a'), 4)

    def test_transaction_put(self):
        self.trans.put('d/a', 5)
        self.assertEquals(self.trans.get('d/a'), 5)
        self.assertDictEqual(self.proxy.get(), {'a': 4, 'b': {'c': 2}})

        self.trans.commit(self.proxy)
        self.assertDictEqual(self.proxy.get(), {'a': 4, 'b': {'c': 2}, 'd': {'a': 5}})

    def test_transaction_delete(self):
        self.trans.delete('a')
        self.assertIsNone(self.trans.get('a'))
        self.assertDictEqual(self.proxy.get(), {'a': 4, 'b': {'c': 2}})

        self.trans.commit(self.proxy)
        self.assertDictEqual(self.proxy.get(), {'b': {'c': 2}})

class ClientTest_PensiveClient(DatabaseDependentTestCase):
    def setUp(self):
        super(ClientTest_PensiveClient, self).setUp()

        self.server.stores[None].put(value={'a': 4})
        self.server.stores.setdefault('a', Store()).put(value={'b': 5})

    def test_client_pensive_index(self):
        self.assertItemsEqual(self.client.index(), [None, 'a'])

    def test_client_pensive_store(self):
        self.assertDictEqual(self.client.store().get(), {'a': 4})
        self.assertDictEqual(self.client.store('a').get(), {'b': 5})
        self.assertDictEqual(self.client.default().get(), {'a': 4})

    def test_client_pensive_delete(self):
        self.client.delete('a')
        self.assertItemsEqual(self.client.index(), [None])
        with self.assertRaises(KeyError):
            self.client.store('a')

    def test_client_pensive_create_default(self):
        store = self.client.create('b', parent=PensiveClient.DEFAULT_STORE)
        self.assertDictEqual(store.get(), {'a': 4})

    def test_client_pensive_create_duplicate(self):
        with self.assertRaises(ValueError):
            store = self.client.create('a')

    def test_client_pensive_create_duplicate_force(self):
        store = self.client.create('a', parent=PensiveClient.DEFAULT_STORE, force=True)
        self.assertDictEqual(store.get(), {'a': 4})

    def test_client_pensive_create_fork(self):
        store = self.client.create('c', 'a')
        self.assertDictEqual(store.get(), {'b': 5})

    def test_client_pensive_create_empty(self):
        store = self.client.create('c', parent=None)
        self.assertIsNone(store.get())

class ClientTest_Coders(DatabaseDependentTestCase):
    def setUp(self):
        super(ClientTest_Coders, self).setUp()

        try:
            import numpy
        except ImportError as e:
            self.skipTest(e)

    def test_client_numpy_ndarray(self):
        self.assertTrue(register_numpy())

        import numpy

        store = self.client.default()
        orig = numpy.arange(10)
        store.put('a', orig)
        result = store.get('a')

        self.assertIs(type(result), numpy.ndarray)
        self.assertTrue(numpy.array_equal(orig, result))

    def test_client_numpy_ndarray_nested(self):
        self.assertTrue(register_numpy())

        import numpy

        store = self.client.default()
        orig = numpy.arange(10)
        store.put('a/b', orig)
        result = store.get('a')['b']

        self.assertIs(type(result), numpy.ndarray)
        self.assertTrue(numpy.array_equal(orig, result))

    def test_client_numpy_matrix(self):
        self.assertTrue(register_numpy())

        import numpy

        store = self.client.default()
        orig = numpy.matrix([[1,2],[3,4]])
        store.put('a', orig)
        result = store.get('a')

        self.assertIs(type(result), numpy.matrix)
        self.assertTrue(numpy.array_equal(orig, result))
