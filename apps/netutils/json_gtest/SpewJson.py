"""
This model will generate something like json objects
Only a subset of rules is enforced, so we can validate
both valid and invalid JSON handling.
"""

import json as JSON
import random

def JSON_end(priv): return ''
def JSON_open_root(priv): return '{'
def JSON_close_root(priv): return '}'
def JSON_open_object(priv): return '{'
def JSON_close_object(priv): return '}'
def JSON_open_pair(priv): return ''
def JSON_close_pair(priv): return ''
def JSON_pair_separator(priv): return ','
def JSON_open_key(priv): return ''
def JSON_close_key(priv): return ''
def JSON_key_value_separator(priv): return ':'
def JSON_open_value(priv): return ''
def JSON_close_value(priv): return ''
def JSON_open_string(priv): return '"'
def JSON_close_string(priv): return '"'
def JSON_control(priv): return priv
def JSON_character(priv): return priv
def JSON_open_number(priv): return ''
def JSON_close_number(priv): return ''
def JSON_true(priv): return 'true'
def JSON_false(priv): return 'false'
def JSON_null(priv): return 'null'
def JSON_open_int(priv): return priv
def JSON_close_int(priv): return ''
def JSON_zero(priv): return '0'
def JSON_digit(priv): return priv
def JSON_open_frac(priv): return '.'
def JSON_close_frac(priv): return ''
def JSON_open_exp(priv): return priv
def JSON_close_exp(priv): return ''
def JSON_open_array(priv): return '['
def JSON_close_array(priv): return ']'
def JSON_open_elements(priv): return ''
def JSON_close_elements(priv): return ''
def JSON_element_separator(priv): return ','
def JSON_whitespace(priv): return priv

#### CONFIGURATION
# Maximum length for generated objects (in characters)
max_length = 50
# By default, we do not allow to close the object until total length reached maximum (lots of invalid objects will be generated)
min_length_is_max_length = True
# Enable to break leading zero rule and allow numbers like: 01, 0001 etc
error_leading_zero = False
# Enable to allow multiple root objects
error_multiple_root_objects = False
# Maximum length of uninterrupted sequence of whitespace
max_whitespace_sequence_length = 3

#### STATE
reset_state = []
json = reset_state[:]
current_string = ''
all_strings = []

def format_json_unit(unit):
    func = unit[0]
    priv = unit[1]
    return func(priv)

def mode():
    if (len(json) == 0):
        return None
    else:
        for i in range(len(json)):
            element = json[-(i + 1)][0]
            if (element != JSON_whitespace):
                return element
        return None # to let root object open

def data():
    if (len(json) == 0):
        return None
    else:
        return json[-1][1]

def count(mode, where=json):
    def reducer(counter, current):
        if (current[0] == mode):
            counter += 1
        return counter
    return reduce(reducer, where, 0)

def inside():
    containers = []
    adders = [JSON_open_root, JSON_open_object, JSON_open_pair, JSON_open_key, JSON_open_value, JSON_open_string, JSON_open_number, JSON_open_int, JSON_open_frac, JSON_open_exp, JSON_open_array, JSON_open_elements]
    removers = [JSON_close_root, JSON_close_object, JSON_close_pair, JSON_close_key, JSON_close_value, JSON_close_string, JSON_close_number, JSON_close_int, JSON_close_frac, JSON_close_exp, JSON_close_array, JSON_close_elements]
    for e in json:
        if (e[0] in adders):
            containers.append(e[0])
        elif (e[0] in removers):
            containers = containers[:-1]
        # no else - is not a container
    if (len(containers) == 0):
        return [None]
    else:
        return containers

def build_json():
    return ''.join(map(format_json_unit, json))

def switch_state(new_state, priv=None):
    global json
    json.append((new_state, priv))
    return build_json()

def End():
    return switch_state(JSON_end)

def EndEnabled():
    return mode() == JSON_close_root

def OpenRoot():
    return switch_state(JSON_open_root)

def OpenRootEnabled():
    current = mode()
    if (current == None):
        return True
    elif (error_multiple_root_objects):
        # One extra root object is enough to demonstrate the bug
        if (current == JSON_close_root and count(JSON_close_root) < 2):
            return True
    return False

def CloseRoot():
    return switch_state(JSON_close_root)

def CloseRootEnabled():
    # It is not allowed to close root object, when we want to force
    # JSON string size. The generator will start new pair inside the
    # same root object
    if (not min_length_is_max_length or len(build_json()) >= max_length):
        if (inside()[-1] == JSON_open_root):
            if (mode() == JSON_close_pair):
                return True
    return False

def OpenObject():
    return switch_state(JSON_open_object)

def OpenObjectEnabled():
    if (mode() == JSON_open_value):
        return True
    return False

def CloseObject():
    return switch_state(JSON_close_object)

def CloseObjectEnabled():
    if (inside()[-1] == JSON_open_object):
        current = mode()
        if (current == JSON_close_pair):
            return True
        elif (current == JSON_open_object):
            # Always allow nested empty objects
            return True
    return False

def OpenPair():
    return switch_state(JSON_open_pair)

def OpenPairEnabled():
    current = mode()
    if (current == JSON_open_root):
        return True
    elif (current == JSON_open_object):
        return True
    elif (current == JSON_pair_separator):
        return True
    return False

def ClosePair():
    return switch_state(JSON_close_pair)

def ClosePairEnabled():
    if (inside()[-1] == JSON_open_pair):
        if (mode() == JSON_close_value):
            return True
    return False

def PairSeparator():
    return switch_state(JSON_pair_separator)

def PairSeparatorEnabled():
    if (mode() == JSON_close_pair):
        return True
    return False

def OpenKey():
    return switch_state(JSON_open_key)

def OpenKeyEnabled():
    if (mode() == JSON_open_pair):
        return True
    return False

def CloseKey():
    return switch_state(JSON_close_key)

def CloseKeyEnabled():
    if (inside()[-1] == JSON_open_key):
        if (mode() == JSON_close_string):
            return True
    return False

def KeyValueSeparator():
    return switch_state(JSON_key_value_separator)

def KeyValueSeparatorEnabled():
    if (mode() == JSON_close_key):
        return True
    return False

def OpenValue():
    return switch_state(JSON_open_value)

def OpenValueEnabled():
    current = mode()
    if (current == JSON_key_value_separator):
        return True
    elif (current == JSON_open_elements):
        return True
    elif (current == JSON_element_separator):
        return True
    return False

def CloseValue():
    return switch_state(JSON_close_value)

def CloseValueEnabled():
    if (inside()[-1] == JSON_open_value):
        current = mode()
        if (current == JSON_close_string):
            return True
        elif (current == JSON_close_number):
            return True
        elif (current == JSON_true):
            return True
        elif (current == JSON_false):
            return True
        elif (current == JSON_null):
            return True
        elif (current == JSON_close_object):
            return True
        elif (current == JSON_close_array):
            return True
    return False

def OpenString():
    global current_string
    current_string = ''
    return switch_state(JSON_open_string)

def OpenStringEnabled():
    current = mode()
    if (current == JSON_open_value):
        return True
    elif (current == JSON_open_key):
        return True
    return False

def CloseString():
    global all_strings
    all_strings.append(current_string)
    return switch_state(JSON_close_string)

def CloseStringEnabled():
    current = mode()
    if (current_string in all_strings):
        return False
    elif (current == JSON_open_string):
        return True
    elif (current == JSON_character):
        return True
    elif (current == JSON_control):
        return True
    return False

def Character():
    global current_string
    character = random.choice(' !#$%&()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[]^_`abcdefghijklmnopqrstuvwxyz{|}~\'')
    current_string += character
    return switch_state(JSON_character, character)

def CharacterEnabled():
    current = mode()
    if (current == JSON_open_string):
        return True
    elif (current == JSON_character):
        return True
    elif (current == JSON_control):
        return True
    return False

def Control():
    global current_string
    control = random.choice(['\\"', '\\\\', '\\/', '\\b', '\\f', '\\n', '\\r', '\\t'])
    current_string += control
    return switch_state(JSON_control, control)

def ControlEnabled():
    return CharacterEnabled()

def OpenNumber():
    return switch_state(JSON_open_number)

def OpenNumberEnabled():
    if (mode() == JSON_open_value):
        return True
    return False

def CloseNumber():
    return switch_state(JSON_close_number)

def CloseNumberEnabled():
    if (inside()[-1] == JSON_open_number):
        current = mode()
        if (current == JSON_close_int):
            return True
        elif (current == JSON_close_frac):
            return True
        elif (current == JSON_close_exp):
            return True
    return False

def OpenInt():
    sign = random.choice(['-', ''])
    return switch_state(JSON_open_int, sign)

def OpenIntEnabled():
    if (mode() == JSON_open_number):
        return True
    return False

def CloseInt():
    return switch_state(JSON_close_int)

def CloseIntEnabled():
    if (inside()[-1] == JSON_open_int):
        current = mode()
        if (current == JSON_zero):
            return True
        elif (current == JSON_digit):
            return True
    return False

def Zero():
    return switch_state(JSON_zero)

def ZeroEnabled():
    current = mode()
    if (current == JSON_open_int):
        return True
    elif (current == JSON_digit):
        return True
    elif (current == JSON_open_frac):
        return True
    elif (current == JSON_open_exp):
        return True
    elif (error_leading_zero and current == JSON_zero):
        return True
    return False

def Digit():
    digit = random.choice('123456789')
    return switch_state(JSON_digit, digit)

def DigitEnabled():
    current = mode()
    if (error_leading_zero):
        if (current == JSON_zero):
            return True
        elif (current == JSON_digit):
            return True
    else:
        if (current == JSON_open_int):
            return True
        elif (current == JSON_digit):
            return True
        elif (inside()[-1] == JSON_open_frac):
            if (current == JSON_zero):
                return True
        elif (inside()[-1] == JSON_open_exp):
            if (current == JSON_zero):
                return True
    return False

def OpenFrac():
    return switch_state(JSON_open_frac)

def OpenFracEnabled():
    if (mode() == JSON_close_int):
        return True
    return False

def CloseFrac():
    return switch_state(JSON_close_frac)

def CloseFracEnabled():
    if (inside()[-1] == JSON_open_frac):
        current = mode()
        if (current == JSON_digit):
            return True
        elif (current == JSON_zero):
            return True
    return False

def OpenExp():
    exp = random.choice(['e', 'e+', 'e-', 'E', 'E+', 'E-'])
    return switch_state(JSON_open_exp, exp)

def OpenExpEnabled():
    current = mode()
    if (current == JSON_close_frac):
        return True
    elif (current == JSON_close_int):
        return True
    return False

def CloseExp():
    return switch_state(JSON_close_exp)

def CloseExpEnabled():
    if (inside()[-1] == JSON_open_exp):
        current = mode()
        if (current == JSON_digit):
            return True
        elif (current == JSON_zero):
            return True
    return False

def JsonTrue():
    return switch_state(JSON_true)

def JsonTrueEnabled():
    if (mode() == JSON_open_value):
        return True
    return False

def JsonFalse():
    return switch_state(JSON_false)

def JsonFalseEnabled():
    if (mode() == JSON_open_value):
        return True
    return False

def Null():
    return switch_state(JSON_null)

def NullEnabled():
    if (mode() == JSON_open_value):
        return True
    return False

def OpenArray():
    return switch_state(JSON_open_array)

def OpenArrayEnabled():
    if (mode() == JSON_open_value):
        return True
    return False

def CloseArray():
    return switch_state(JSON_close_array)

def CloseArrayEnabled():
    current = mode()
    if (current == JSON_open_array):
        return True
    elif (current == JSON_close_elements):
        return True
    return False

def OpenElements():
    return switch_state(JSON_open_elements)

def OpenElementsEnabled():
    if (mode() == JSON_open_array):
        return True

def CloseElements():
    return switch_state(JSON_close_elements)

def CloseElementsEnabled():
    if (inside()[-1] == JSON_open_elements):
        if (mode() == JSON_close_value):
            return True
    return False

def ElementSeparator():
    return switch_state(JSON_element_separator)

def ElementSeparatorEnabled():
    if (inside()[-1] == JSON_open_elements):
        if (mode() == JSON_close_value):
            return True
    return False

def Whitespace():
    whitespace = random.choice([' ', '\t', '\r', '\n'])
    return switch_state(JSON_whitespace, whitespace)

def WhitespaceEnabled():
    if (count(JSON_whitespace, json[-max_whitespace_sequence_length:]) == max_whitespace_sequence_length):
        # Let whitespace group be no longer than max_whitespace_sequence_length pieces
        return False
    current = mode()
    containers = inside()
    if (JSON_open_string in containers):
        return False
    elif (JSON_open_number in containers):
        return False
    elif (current == JSON_end):
        return False
    return True

###################################

def StateFilter():
    return len(build_json()) <= max_length

def state_invariant():
    try:
        JSON.loads(build_json())
        return True
    except ValueError:
        return False

def Reset():
    global json, current_string, all_strings
    json = reset_state[:]
    current_string = ''
    all_strings = reset_state[:]

def Accepting():
    return state_invariant()

state = ('json','current_string','all_strings')
actions = (
        End,
        OpenRoot, CloseRoot,
        OpenObject, CloseObject,
        OpenPair, ClosePair, PairSeparator,
        OpenKey, CloseKey,
        KeyValueSeparator,
        OpenValue, CloseValue,
        OpenString, CloseString,
        Character, Control,
        OpenNumber, CloseNumber,
        OpenInt, CloseInt,
        Zero, Digit,
        OpenFrac, CloseFrac,
        OpenExp, CloseExp,
        JsonTrue, JsonFalse, Null,
        OpenArray, CloseArray,
        OpenElements, CloseElements,
        ElementSeparator,
        Whitespace
        )
enablers = {
        End:(EndEnabled,),
        OpenRoot:(OpenRootEnabled,),
        CloseRoot:(CloseRootEnabled,),
        OpenObject:(OpenObjectEnabled,),
        CloseObject:(CloseObjectEnabled,),
        OpenPair:(OpenPairEnabled,),
        ClosePair:(ClosePairEnabled,),
        PairSeparator:(PairSeparatorEnabled,),
        OpenKey:(OpenKeyEnabled,),
        CloseKey:(CloseKeyEnabled,),
        KeyValueSeparator:(KeyValueSeparatorEnabled,),
        OpenValue:(OpenValueEnabled,),
        CloseValue:(CloseValueEnabled,),
        OpenString:(OpenStringEnabled,),
        CloseString:(CloseStringEnabled,),
        Character:(CharacterEnabled,),
        Control:(ControlEnabled,),
        OpenNumber:(OpenNumberEnabled,),
        CloseInt:(CloseIntEnabled,),
        OpenInt:(OpenIntEnabled,),
        OpenFrac:(OpenFracEnabled,),
        CloseFrac:(CloseFracEnabled,),
        OpenExp:(OpenExpEnabled,),
        CloseExp:(CloseExpEnabled,),
        CloseNumber:(CloseNumberEnabled,),
        Zero:(ZeroEnabled,),
        Digit:(DigitEnabled,),
        JsonTrue:(JsonTrueEnabled,),
        JsonFalse:(JsonFalseEnabled,),
        Null:(NullEnabled,),
        OpenArray:(OpenArrayEnabled,),
        CloseArray:(CloseArrayEnabled,),
        OpenElements:(OpenElementsEnabled,),
        CloseElements:(CloseElementsEnabled,),
        ElementSeparator:(ElementSeparatorEnabled,),
        Whitespace:(WhitespaceEnabled,)
        }

