from unittest import TestCase

from src.pensive.core import MemoryStore

class MemoryStore_Get(TestCase):

    def setUp(self):
        self.store = MemoryStore({
            'empty': None,
            'path\\/': 4,
            'value': 2,
            'nested': {
                'a': 1,
                'b': 2,
            },
            'list': [1, 2, 3, {'c': 3}],
        })

    def test_get_empty(self):
        self.assertIsNone(self.store.get('empty'))

    def test_get_nonempty(self):
        self.assertEqual(self.store.get('value'), 2)

    def test_get_escaped_path(self):
        self.assertEqual(self.store.get('path\\/'), 4)

    def test_get_nested_path(self):
        self.assertEqual(self.store.get('nested/a'), 1)

    def test_get_partial_path(self):
        self.assertDictEqual(self.store.get('nested'), {'a': 1, 'b': 2})

    def test_get_nonexisting(self):
        self.assertRaises(KeyError, lambda: self.store.get('nonexisting'))

    def test_get_nonexisting2(self):
        self.assertRaises(KeyError, lambda: self.store.get(''))

    def test_get_list(self):
        self.assertRaises(TypeError, lambda: self.store.get('list/a'))

    def test_get_list_nested(self):
        self.assertRaises(KeyError, lambda: self.store.get('list/a/b'))

    def tearDown(self):
        pass

class MemoryStore_Put(TestCase):

    def setUp(self):
        self.store = MemoryStore()

    def test_put_simple(self):
        self.store.put('a', 4)
        self.assertDictEqual(self.store._root, {'a': 4})

    def test_put_nested(self):
        self.store.put('a/b/c', 4)
        self.assertDictEqual(self.store._root, {'a': {'b': {'c': 4}}})

    def test_put_dict(self):
        self.store.put('a', {'b': 2})
        self.assertDictEqual(self.store._root, {'a': {'b': 2}})

    def test_put_nested_existing(self):
        self.store.put('a/b/c', 4)
        self.store.put('a/b/c', 4)
        self.assertDictEqual(self.store._root, {'a': {'b': {'c': 4}}})

    def test_put_list(self):
        self.store.put('list', [])
        self.assertRaises(TypeError, lambda: self.store.put('list/a', 2))

    def test_put_list_nested(self):
        self.store.put('list', [])
        self.assertRaises(KeyError, lambda: self.store.put('list/a/b', 2))

    def tearDown(self):
        pass

class MemoryStore_Delete(TestCase):

    def setUp(self):
        self.store = MemoryStore({'a': None, 'b': {'c': 2}})

    def test_delete_simple(self):
        self.store.delete('a')
        self.assertDictEqual(self.store._root, {'b': {'c': 2}})

    def test_delete_nested(self):
        self.store.delete('b/c')
        self.assertDictEqual(self.store._root, {'a': None, 'b': {}})

    def test_delete_dict(self):
        self.store.delete('b')
        self.assertDictEqual(self.store._root, {'a': None})

    def test_delete_nonexisting(self):
        self.assertRaises(KeyError, lambda: self.store.delete('random'))

    def tearDown(self):
        pass

class MemoryStore_Find(TestCase):

    def setUp(self):
        self.store = MemoryStore({
            'empty': None,
            'value': 2,
            'nested': {
                'a': 1,
                'b': 2,
            },
            'list': [1, 2, 3, {'c': 3}],
        })

    def test_find(self):
        self.assertListEqual(self.store.find(''), [('list', None), ('empty', None), ('value', None), ('nested', ([('a', None), ('b', None)])) ])

    def tearDown(self):
        pass
