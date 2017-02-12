# pylint: disable=line-too-long,missing-docstring,invalid-name,protected-access

from unittest import TestCase

from pensive.core import Store

class Store_Get(TestCase):

    def setUp(self):
        self.store = Store({
            'empty': None,
            'path\\/': 4,
            'value': 2,
            'nested': {
                'a': 1,
                'b': 2,
            },
            'list': [1, 2, 3, {'c': 3}],
        })

    def test_store_get_empty(self):
        self.assertIsNone(self.store.get('empty'))

    def test_store_get_empty2(self):
        self.assertIsNone(self.store.get('empty/reallyempty'))

    def test_store_get_nonempty(self):
        self.assertEqual(self.store.get('value'), 2)

    def test_store_get_escaped_path(self):
        self.assertEqual(self.store.get('path\\/'), 4)

    def test_store_get_nested_path(self):
        self.assertEqual(self.store.get('nested/a'), 1)

    def test_store_get_partial_path(self):
        self.assertDictEqual(self.store.get('nested'), {'a': 1, 'b': 2})

    def test_store_get_nonexisting(self):
        self.assertIsNone(self.store.get('nonexisting'))

    def test_store_get_nonexisting_strict(self):
        with self.assertRaises(KeyError):
            self.store.get('nonexisting', strict=True)

    def test_store_get_nonexisting_strict2(self):
        with self.assertRaises(KeyError):
            self.store.get('value/a', strict=True)

    def test_store_get_nonexisting_strict3(self):
        with self.assertRaises(KeyError):
            self.store.get('nested/c', strict=True)

    def test_store_get_nooverwrite(self):
        self.store.get('nested/a/asdf')
        self.assertDictEqual(self.store.get('nested'), {'a': 1, 'b': 2})

    def tearDown(self):
        pass

class Store_Put(TestCase):

    def setUp(self):
        self.store = Store()

    def test_store_put_simple(self):
        self.store.put('a', 4)
        self.assertDictEqual(self.store._serialize(), {'a': 4})

    def test_store_put_nested(self):
        self.store.put('a/b/c', 4)
        self.assertDictEqual(self.store._serialize(), {'a': {'b': {'c': 4}}})

    def test_store_put_dict(self):
        self.store.put('a', {'b': 2})
        self.assertDictEqual(self.store._serialize(), {'a': {'b': 2}})

    def test_store_put_nested_existing(self):
        self.store.put('a/b/c', 4)
        self.store.put('a/b/c', 4)
        self.assertDictEqual(self.store._serialize(), {'a': {'b': {'c': 4}}})

    def test_store_put_overwrite(self):
        self.store.put('a/b', 3)
        self.store.put('a/b/c', 4)
        self.assertDictEqual(self.store._serialize(), {'a': {'b': {'c': 4}}})

    def test_store_put_overwrite_strict(self):
        self.store.put('a/b', 3)
        with self.assertRaises(KeyError):
            self.store.put('a/b/c', 4, strict=True)

    def test_store_put_overwrite_strict2(self):
        self.store.put('a/b', 3)
        with self.assertRaises(KeyError):
            self.store.put('a', 4, strict=True)

    def test_store_put_deserialize(self):
        self.store.put('a', {'b': {'c': 4}})
        self.assertEqual(self.store.get('a/b/c'), 4)

    def tearDown(self):
        pass

class Store_Delete(TestCase):

    def setUp(self):
        self.store = Store({'a': 4, 'b': {'c': 2, 'd': 3}})

    def test_store_delete_simple(self):
        self.store.delete('a')
        self.assertDictEqual(self.store._serialize(), {'b': {'c': 2, 'd': 3}})

    def test_store_delete_nested(self):
        self.store.delete('b/c')
        self.assertDictEqual(self.store._serialize(), {'a': 4, 'b': {'d': 3}})

    def test_store_delete_double(self):
        self.store.delete('b/c')
        self.store.delete('b/d')
        self.assertDictEqual(self.store._serialize(), {'a': 4})

    def test_store_delete_dict(self):
        self.store.delete('b')
        self.assertDictEqual(self.store._serialize(), {'a': 4})

    def test_store_delete_nonexisting(self):
        self.store.delete('nonexisting')
        self.assertDictEqual(self.store._serialize(), {'a': 4, 'b': {'c': 2, 'd': 3}})

    def tearDown(self):
        pass

class Store_Index(TestCase):

    def setUp(self):
        self.store = Store({
            'empty': None,
            'value': 2,
            'nested': {
                'a': 1,
                'b': 2,
            },
            'list': [1, 2, 3, {'c': 3}],
        })

    def test_store_index(self):
        self.assertDictEqual(self.store.index(), {'value': {}, 'nested': {'a': {}, 'b': {}}, 'list': {}})

    def test_store_index_depth2(self):
        self.assertDictEqual(self.store.index(depth=2), {'value': {}, 'nested': {'a': {}, 'b': {}}, 'list': {}})

    def test_store_index_depth1(self):
        self.assertDictEqual(self.store.index(depth=1), {'value': {}, 'nested': {}, 'list': {}})

    def test_store_index_depth0(self):
        self.assertDictEqual(self.store.index(depth=0), {})

    def test_store_index_nested(self):
        self.assertDictEqual(self.store.index('nested'), {'a': {}, 'b': {}})

    def test_store_index_nested_depth(self):
        self.assertDictEqual(self.store.index('nested', depth=0), {})

    def test_store_index_nonexisting(self):
        self.assertIsNone(self.store.index('nonexisting'))

    def test_store_index_value(self):
        self.assertDictEqual(self.store.index('value'), {})

    def test_store_index_empty(self):
        self.assertIsNone(self.store.index('value/empty'))

    def tearDown(self):
        pass

class Store_Cull(TestCase):

    def setUp(self):
        self.store = Store({
            'empty': None,
            'value': 0,
            'nested': {
                'a': 1,
                'b': 2,
            },
            'list': [1, 2, 3, {'c': 3}],
            'empty_nested' : {
                'a': {
                    'b': None
                }
            }
        })

    def test_store_cull(self):
        self.store.cull()
        self.assertDictEqual(self.store.get(), {'value': 0, 'nested': {'a': 1, 'b': 2}, 'list': [1, 2, 3, {'c': 3}]})

    def test_store_cull2(self):
        self.assertItemsEqual(['empty', 'value', 'nested', 'list', 'empty_nested'], self.store._children.keys())
        self.store.cull()
        self.assertItemsEqual(['value', 'nested', 'list'], self.store._children.keys())

    def tearDown(self):
        pass

class Store_Empty(TestCase):

    def setUp(self):
        self.store = Store({
            'empty': None,
            'value': 0,
            'nested': {
                'a': 1,
                'b': 2,
            },
            'empty_nested' : {
                'a': {
                    'b': None
                }
            }
        })

    def test_store_empty(self):
        self.assertFalse(self.store.is_empty())

    def test_store_empty2(self):
        self.store.delete('nested')
        self.assertFalse(self.store.is_empty())

    def test_store_empty3(self):
        self.store.delete('nested')
        self.store.delete('value')
        self.assertTrue(self.store.is_empty())

    def tearDown(self):
        pass

class Store_Fork(TestCase):

    def setUp(self):
        self.store = Store({
            'a': {
                'b1': 1,
                'b2': [1, 2],
                'b3': {'c': [3, 4]},
            }
        })

    def test_store_copy(self):
        copy = self.store.fork()
        self.assertDictEqual(copy._serialize(), {'a': {'b1': 1, 'b2': [1, 2], 'b3': {'c': [3, 4]}}})
        self.assertIsNot(copy._children, self.store._children)
        self.assertIsNot(copy._children['a'], self.store._children['a'])
        self.assertIs(copy.get('a/b2'), self.store.get('a/b2'))
        self.assertIsNot(copy._children['a']._children['b3'], self.store._children['a']._children['b3'])
        self.assertIs(copy.get('a/b3/c'), self.store.get('a/b3/c'))

    def test_store_copy_empty(self):
        copy = Store().fork()
        self.assertIsNone(copy.get())

    def test_store_copy_empty2(self):
        self.store.delete('a')
        copy = self.store.fork()
        self.assertIsNone(copy.get())

    # def test_store_copy_nested(self):
    #     copy = self.store.fork('a/b3')
    #     self.assertDictEqual(copy._root, {'c': [3, 4]})
    #     self.assertIsNot(copy._root, self.store._root)
    #     self.assertIs(copy._root['c'], self.store._root['a']['b3']['c'])

    def tearDown(self):
        pass

class Store_Flatten(TestCase):

    def setUp(self):
        self.store = Store({
            'a': {
                'b1': 1,
                'b2': 2,
                'b3': {'c': 3},
                'b4': None
            }
        })

    def test_store_flatten(self):
        self.assertDictEqual(self.store.flatten(), {'a/b1': 1, 'a/b2': 2, 'a/b3/c': 3})

    def test_store_flatten_strict(self):
        self.assertDictEqual(self.store.flatten(strict=True), {'a/b1': 1, 'a/b2': 2, 'a/b3/c': 3, 'a/b4': None})

    def tearDown(self):
        pass
