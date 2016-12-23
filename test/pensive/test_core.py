from unittest import TestCase

from src.pensive.core import Store

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

    def test_get_empty(self):
        self.assertIsNone(self.store.get('empty'))

    def test_get_empty2(self):
        self.assertIsNone(self.store.get('empty/reallyempty'))

    def test_get_nonempty(self):
        self.assertEqual(self.store.get('value'), 2)

    def test_get_escaped_path(self):
        self.assertEqual(self.store.get('path\\/'), 4)

    def test_get_nested_path(self):
        self.assertEqual(self.store.get('nested/a'), 1)

    def test_get_partial_path(self):
        self.assertDictEqual(self.store.get('nested'), {'a': 1, 'b': 2})

    def test_get_nonexisting(self):
        self.assertIsNone(self.store.get('nonexisting'))

    def test_get_nooverwrite(self):
        self.store.get('nested/a/asdf')
        self.assertDictEqual(self.store.get('nested'), {'a': 1, 'b': 2})

    def tearDown(self):
        pass

class Store_Put(TestCase):

    def setUp(self):
        self.store = Store()

    def test_put_simple(self):
        self.store.put('a', 4)
        self.assertDictEqual(self.store._serialize(), {'a': 4})

    def test_put_nested(self):
        self.store.put('a/b/c', 4)
        self.assertDictEqual(self.store._serialize(), {'a': {'b': {'c': 4}}})

    def test_put_dict(self):
        self.store.put('a', {'b': 2})
        self.assertDictEqual(self.store._serialize(), {'a': {'b': 2}})

    def test_put_nested_existing(self):
        self.store.put('a/b/c', 4)
        self.store.put('a/b/c', 4)
        self.assertDictEqual(self.store._serialize(), {'a': {'b': {'c': 4}}})

    def test_put_overwrite(self):
        self.store.put('a/b', 3)
        self.store.put('a/b/c', 4)
        self.assertDictEqual(self.store._serialize(), {'a': {'b': {'c': 4}}})

    def tearDown(self):
        pass

class Store_Delete(TestCase):

    def setUp(self):
        self.store = Store({'a': 4, 'b': {'c': 2, 'd': 3}})

    def test_delete_simple(self):
        self.store.delete('a')
        self.assertDictEqual(self.store._serialize(), {'b': {'c': 2, 'd': 3}})

    def test_delete_nested(self):
        self.store.delete('b/c')
        self.assertDictEqual(self.store._serialize(), {'a': 4, 'b': {'d': 3} })

    def test_delete_double(self):
        self.store.delete('b/c')
        self.store.delete('b/d')
        self.assertDictEqual(self.store._serialize(), {'a': 4 })

    def test_delete_dict(self):
        self.store.delete('b')
        self.assertDictEqual(self.store._serialize(), {'a': 4})

    def test_delete_nonexisting(self):
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

    def test_index(self):
        self.assertDictEqual(self.store.index(), { 'value': {}, 'nested': {'a': {}, 'b': {}}, 'list': {}})

    def test_index_nested(self):
        self.assertDictEqual(self.store.index('nested'), {'a': {}, 'b': {}})

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

    def test_cull(self):
        self.store.cull()
        self.assertDictEqual(self.store.get(), { 'value': 0, 'nested': {'a': 1, 'b': 2}, 'list': [1, 2, 3, {'c': 3}]})

    def test_cull2(self):
        self.assertItemsEqual(self.store._children.keys(), ['empty', 'value', 'nested', 'list', 'empty_nested'])
        self.store.cull()
        self.assertItemsEqual(self.store._children.keys(), ['value', 'nested', 'list'])

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

    def test_empty(self):
        self.assertFalse(self.store.is_empty())

    def test_empty2(self):
        self.store.delete('nested')
        self.assertFalse(self.store.is_empty())

    def test_empty3(self):
        self.store.delete('nested')
        self.store.delete('value')
        self.assertTrue(self.store.is_empty())

    def tearDown(self):
        pass


# class Store_Fork(TestCase):

#     def setUp(self):
#         self.store = Store({
#             'a': {
#                 'b1': 1,
#                 'b2': [1, 2],
#                 'b3': {'c': [3, 4]},
#             }
#         })

#     def test_copy(self):
#         copy = self.store.fork()
#         self.assertDictEqual(copy._root, {'a': {'b1': 1, 'b2': [1, 2], 'b3': {'c': [3, 4]}}})
#         self.assertIsNot(copy._root, self.store._root)
#         self.assertIsNot(copy._root['a'], self.store._root['a'])
#         self.assertIs(copy._root['a']['b2'], self.store._root['a']['b2'])
#         self.assertIsNot(copy._root['a']['b3'], self.store._root['a']['b3'])
#         self.assertIs(copy._root['a']['b3']['c'], self.store._root['a']['b3']['c'])

#     def test_copy_nested(self):
#         copy = self.store.fork('a/b3')
#         self.assertDictEqual(copy._root, {'c': [3, 4]})
#         self.assertIsNot(copy._root, self.store._root)
#         self.assertIs(copy._root['c'], self.store._root['a']['b3']['c'])

#     def tearDown(self):
#         pass
