''' Common functions:
    * class SimpleNamespace
    * def dict2class
    * read_yaml_file
'''

import yaml


class SimpleNamespace:
    ''' This is the same as `from type import SimpleNamespace` in Python3 '''

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __repr__(self):
        keys = sorted(self.__dict__)
        items = ("{}={!r}".format(k, self.__dict__[k]) for k in keys)
        return "{}({})".format(type(self).__name__, ", ".join(items))

    def __eq__(self, other):
        return self.__dict__ == other.__dict__


def dict2class(d):
    ''' Convert a dictionary to a class.
    The keys in the dictionary must be the type `str`.
    '''
    args = SimpleNamespace()
    args.__dict__.update(**d)
    return args


def read_yaml_file(filepath):
    ''' Read contents from the yaml file.
    Output:
        data_dict {dict}: contents of the yaml file.
            The keys of the dict are `str` type.
    '''
    with open(filepath, 'r') as stream:
        data_dict = yaml.safe_load(stream)
    return data_dict
