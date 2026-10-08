##############################################################################
#            gdb_printers.py
#
#  Copyright  2010  Pieter Collins
#    Some of these are based on pretty-printers for libstc++.
#    Copyright (C) 2008, 2009, 2010 Free Software Foundation, Inc.
##############################################################################

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


# Pretty-printers for Ariadne
# To use these printers, put this file in a directory $TOP/ariadne_gdb_printers.py
# and add the following to you .gdbinit file, replacing $TOP by the top-level directory
#import sys
#sys.path.insert(0, '$TOP/')
#from pyariadne/gdb_printers import register_ariadne_printers
#register_ariadne_printers (None)
#end

import gdb
import itertools
import re


def strip_namespace(string):
    string=str(string)
    if string[:9]=='Ariadne::':
        return string[9:]
    elif string[:5]=='std::':
        return string[5:]
    elif string[:7]=='boost::':
        return string[7:]
    else:
        return string

class ListPrinter:
    class _iterator:
        def __init__ (self, start, finish):
            self.item = start
            self.finish = finish
            self.count = 0

        def __iter__(self):
            return self

        def next(self):
            count = self.count
            self.count = self.count + 1
            if self.item == self.finish:
                raise StopIteration
            elt = self.item.dereference()
            self.item = self.item + 1
            return ('[%d]' % count, elt)

    def __init__(self, typename, val):
        self.typename = typename
        self.val = val

    def children(self):
        return self._iterator(self.val['_M_impl']['_M_start'],
                              self.val['_M_impl']['_M_finish'])

    def to_string(self):
        start = self.val['_M_impl']['_M_start']
        finish = self.val['_M_impl']['_M_finish']
        elementtypename = strip_namespace(self.val.type.template_argument(0))
        return ('%s<%s>[%d]' % (self.typename, elementtypename, int (finish - start)))

    def display_hint(self):
        return 'array'


class RbtreeIterator:
    def __init__(self, rbtree):
        self.size = rbtree['_M_t']['_M_impl']['_M_node_count']
        self.node = rbtree['_M_t']['_M_impl']['_M_header']['_M_left']
        self.count = 0

    def __iter__(self):
        return self

    def __len__(self):
        return int (self.size)

    def next(self):
        if self.count == self.size:
            raise StopIteration
        result = self.node
        self.count = self.count + 1
        if self.count < self.size:
            # Compute the next node.
            node = self.node
            if node.dereference()['_M_right']:
                node = node.dereference()['_M_right']
                while node.dereference()['_M_left']:
                    node = node.dereference()['_M_left']
            else:
                parent = node.dereference()['_M_parent']
                while node == parent.dereference()['_M_right']:
                    node = parent
                    parent = parent.dereference()['_M_parent']
                if node.dereference()['_M_right'] != parent:
                    node = parent
            self.node = node
        return result

class MapPrinter:
    "Print a std::map or std::multimap"

    # Turn an RbtreeIterator into a pretty-print iterator.
    class _iter:
        def __init__(self, rbiter, type):
            self.rbiter = rbiter
            self.count = 0
            self.type = type

        def __iter__(self):
            return self

        def next(self):
            self.pair = self.rbiter.next().cast(self.type).dereference()['_M_value_field']
            result = ('%s:' % self.pair['first'], self.pair['second'])
            self.count = self.count + 1
            return result

    def __init__ (self, typename, val):
        self.typename = typename
        self.val = val

    def to_string (self):
        keytypename = strip_namespace(self.val.type.template_argument(0))
        valuetypename = strip_namespace(self.val.type.template_argument(1))
        return '%s<%s,%s>[%d]' % (self.typename, keytypename, valuetypename, len (RbtreeIterator (self.val)))

    def children (self):
        keytype = self.val.type.template_argument(0).const()
        valuetype = self.val.type.template_argument(1)
        nodetype = gdb.lookup_type('std::_Rb_tree_node< std::pair< %s, %s > >' % (keytype, valuetype))
        nodetype = nodetype.pointer()
        return self._iter (RbtreeIterator (self.val), nodetype)

    def display_hint (self):
        return 'set' # Seems to work better than 'map' hint

class SetPrinter:
    # Turn an RbtreeIterator into a pretty-print iterator.
    class _iter:
        def __init__(self, rbiter, type):
            self.rbiter = rbiter
            self.count = 0
            self.type = type

        def __iter__(self):
            return self

        def next(self):
            item = self.rbiter.next()
            item = item.cast(self.type).dereference()['_M_value_field']
            result = ('%s' % item)
            self.count = self.count + 1
            return result

    def __init__ (self, typename, val):
        self.typename = typename
        self.val = val

    def __iter__ (self):
        keytype = self.val.type.template_argument(0)
        nodetype = gdb.lookup_type('std::_Rb_tree_node< %s >' % keytype).pointer()
        return self._iter(RbtreeIterator (self.val), nodetype)

    def to_string (self):
        element_type=self.val.type.template_argument(0)
        element_typename=strip_namespace(str(element_type))
        res = '%s<%s>[%d]' % (self.typename, element_typename, len (RbtreeIterator (self.val)))
        res=res + '{ '
        sep = ''
        for elmt in self:
            res = res + sep + str(elmt)
            sep = ', '
        return res + ' }'




class FloatPrinter:
    def __init__ (self, val):
        self.val = val

    def to_string (self):
        return self.val['v']

class IntervalPrinter:
    def __init__ (self, val):
        self.val = val

    def to_string (self):
        return '[%s:%s]' % (self.val['l'],self.val['u'])

class NumericVectorPrinter:
    def __init__(self, typename, val):
        self.typename = typename
        self.val = val

    def to_string(self):
        element_type=self.val.type.template_argument(0)
        element_typename=strip_namespace(element_type)
        array=self.val['data_']
        size=array['size_']
        begin=array['data_']
        res = 'Vector<%s>' % element_typename
        res = res + "[%d]" % size
        res = res + "{"
        for i in range(int(size)):
            if(i!=0): res=res+","
            res=res+str((begin+i).dereference())
        res = res + "}"
        return res


class ArrayVectorPrinter:
    def __init__(self, typename, val):
        self.typename = typename
        self.val = val

    class _iterator:
        def __init__ (self, size, start):
            self.size = size
            self.item = start
            self.count = 0

        def __iter__(self):
            return self

        def next(self):
            if self.count == self.size:
                raise StopIteration
            count=self.count
            elt = self.item.dereference()
            self.count = self.count + 1
            self.item = self.item + 1
            return ('[%d]' % count, elt)

    def children(self):
        return self._iterator(self.val['data_']['size_'],self.val['data_']['data_'])

    def to_string(self):
        element_type=self.val.type.template_argument(0)
        element_typename=strip_namespace(element_type)
        array=self.val['data_']
        size=array['size_']
        begin=array['data_']
        #Strip "Ariadne::" from element type name below
        res = 'Vector<%s>' % element_typename
        res = res + '[%d]' % size
        return res

    def display_hint(self):
        return 'array'

class ExpansionPrinter:
    def __init__(self, typename, val):
        self.typename = typename
        self.val = val

    def to_string(self):
        byte_type=gdb.lookup_type('unsigned char')
        data_type=self.val.type.template_argument(0)
        byte_pointer = byte_type.pointer()
        data_pointer = data_type.pointer()

        argument_size = self.val['_argument_size']
        sizeof_word = self.val['sizeof_word']
        sizeof_index = ((argument_size+sizeof_word)/sizeof_word)*sizeof_word
        sizeof_data = self.val['sizeof_data']
        sizeof_element = sizeof_index + sizeof_data

        coefficients_start = self.val['_coefficients']['_M_impl']['_M_start'].cast(byte_pointer)
        coefficients_finish = self.val['_coefficients']['_M_impl']['_M_finish'].cast(byte_pointer)
        coefficients_size = coefficients_finish - coefficients_start

        number_of_elements = coefficients_size / sizeof_element
        res = ''
        #res = res + 'Expansion<'+strip_namespace(data_type)+">"
        #res = res + "{ argument_size = " + str(argument_size) + ", number_of_nonzeros = " + str(number_of_elements) + " } "
        res = res + '{'
        for i in range(0,number_of_elements):
            element_start = coefficients_start + i * sizeof_element
            if i!=0: res = res + '; '
            for j in range(argument_size):
                if j!=0: res = res + ","
                res = res + str(int((element_start+j).dereference()))
            res = res + ':' + str((element_start+sizeof_index).cast(data_pointer).dereference())
        res = res + '}'
        return res

class TaylorModelPrinter:
    def __init__(self, val):
        self.val = val

    def children(self):
        #items = [ ('_expansion',self.val['_expansion']), \
        #          ('_error',self.val['_error']), \
        #          ('_sweep_threshold',self.val['_accuracy_ptr']['px'].dereference()['_sweep_threshold']), \
        #          ('_maximum_degree',self.val['_accuracy_ptr']['px'].dereference()['_maximum_degree']) ]
        items = [ ('_expansion',self.val['_expansion']), \
                  ('_error',self.val['_error']), \
                  ('_accuracy',self.val['_accuracy_ptr']['px'].dereference()) ]
        return iter(items)

    def to_string(self):
        return 'TaylorModel'

class TaylorModelAccuracyPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        return '{ _sweep_threshold=%s, _maximum_degree=%d }' % (self.val['_sweep_threshold'], int(self.val['_maximum_degree']) )

class ScalarTaylorFunctionPrinter:
    def __init__(self, val):
        self.val = val

    def children(self):
        items = [ ('_domain',self.val['_domain']), \
                  ('_expansion',self.val['_model']['_expansion']), \
                  ('_error',self.val['_model']['_error']), \
                  ('_accuracy',self.val['_model']['_accuracy_ptr']['px'].dereference()) ]
        return iter(items)

    def to_string(self):
        return 'ScalarTaylorFunction'

class TaylorModelPrinter:
    def __init__(self, val):
        self.val = val

    def children(self):
        #items = [ ('_expansion',self.val['_expansion']), \
        #          ('_error',self.val['_error']), \
        #          ('_sweep_threshold',self.val['_accuracy_ptr']['px'].dereference()['_sweep_threshold']), \
        #          ('_maximum_degree',self.val['_accuracy_ptr']['px'].dereference()['_maximum_degree']) ]
        items = [ ('_expansion',self.val['_expansion']), \
                  ('_error',self.val['_error']), \
                  ('_accuracy',self.val['_accuracy_ptr']['px'].dereference()) ]
        return iter(items)

    def to_string(self):
        return 'TaylorModel'

class DiscreteEventPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        return self.val['_id']


def register_ariadne_printers (obj):
    "Register Ariadne pretty-printers with objfile Obj."

    if obj == None:
        obj = gdb

    obj.pretty_printers.append (lookup_function)

def lookup_function (val):
    "Look-up and return a pretty-printer that can print val."

    # Get the type.
    type = val.type

    # If it points to a reference, get the reference.
    if type.code == gdb.TYPE_CODE_REF:
        type = type.target ()

    # Get the unqualified type, stripped of typedefs.
    type = type.unqualified ().strip_typedefs ()

    # Get the type name.
    typename = type.tag
    if typename == None:
        return None

    # Iterate over local dictionary of types to determine
    # if a printer is registered for that type.  Return an
    # instantiation of the printer if found.
    for function in pretty_printers_dict:
        if function.search (typename):
            return pretty_printers_dict[function] (val)

    # Cannot find a pretty printer.  Return None.
    return None

def build_ariadne_dictionary ():
    pretty_printers_dict[re.compile('^Ariadne::List<.*>$')] = lambda val: ListPrinter("List", val)
    pretty_printers_dict[re.compile('^Ariadne::Set<.*>$')] = lambda val: SetPrinter("Set", val)
    pretty_printers_dict[re.compile('^Ariadne::Map<.*>$')] = lambda val: MapPrinter("Map", val)
    pretty_printers_dict[re.compile('^Ariadne::Float$')] = lambda val: FloatPrinter(val)
    pretty_printers_dict[re.compile('^Ariadne::Interval$')] = lambda val: IntervalPrinter(val)
    pretty_printers_dict[re.compile('^Ariadne::Vector<Ariadne::Float>$')] = lambda val: NumericVectorPrinter("Vector", val)
    pretty_printers_dict[re.compile('^Ariadne::Vector<Ariadne::Interval>$')] = lambda val: NumericVectorPrinter("Vector", val)
    pretty_printers_dict[re.compile('^Ariadne::Vector<Ariadne::TaylorModel>$')] = lambda val: ArrayVectorPrinter("Vector", val)
    pretty_printers_dict[re.compile('^Ariadne::Expansion<.*>$')] = lambda val: ExpansionPrinter("Expansion", val)
    pretty_printers_dict[re.compile('^Ariadne::TaylorModel$')] = lambda val: TaylorModelPrinter(val)
    pretty_printers_dict[re.compile('^Ariadne::TaylorModel::Accuracy$')] = lambda val: TaylorModelAccuracyPrinter(val)
    pretty_printers_dict[re.compile('^Ariadne::ScalarTaylorFunction$')] = lambda val: ScalarTaylorFunctionPrinter(val)
    pretty_printers_dict[re.compile('^Ariadne::Box')] = lambda val: NumericVectorPrinter("Box",val)
    pretty_printers_dict[re.compile('^Ariadne::DiscreteEvent$')] = lambda val: DiscreteEventPrinter(val)

pretty_printers_dict = {}

build_ariadne_dictionary ()
