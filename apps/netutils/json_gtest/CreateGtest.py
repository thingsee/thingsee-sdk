#!/usr/bin/env python

import CollectTraces
import md5
import sys
import json
from ObjWalk import objwalk
import math

print """\
#include <limits.h>
#include "gtest/gtest.h"
extern "C" {
#include "cJSON.h"
}
"""

def safe_string(string):
    if (type(string) == str):
        return json.dumps(string)[1:-1]
    elif(type(string) == unicode):
        return safe_string(string.encode('ascii', 'ignore'))

def c_string(string):
    return safe_string(string)#.replace('"', '\\"')

def format_path(path):
    string = []
    for p in path:
        if (type(p) == int):
            string.append(str(p))
        else:
            string.append("'" + safe_string(p) + "'")
    return ':'.join(string)

imported = []
for module in sys.argv[1:]:
    imported.append(__import__(module))

unique = CollectTraces.find_unique(imported)
for obj in unique:
    m = md5.new()
    for action in obj['actions']:
        m.update(action)
    print 'TEST(cJSON, subject_' + m.hexdigest() + ') {'
    for action in obj['actions']:
        print '\t// ' + action
    print '\t// JSON: \'' + safe_string(obj['json']) + '\''
    print '\tconst char* json = "' + c_string(obj['json']) + '";';
    print '\tstd::cerr << "Plain JSON string from model: " << "' + c_string(safe_string(obj['json'])) + '" << std::endl;'
    print '\tcJSON* object = cJSON_Parse(json);'
    json_object = obj['object']
    if (json_object != None):
        print '\tASSERT_TRUE(NULL != object);'
        print '\tchar* formatted = cJSON_Print(object);'
        print '\tstd::cerr << "cJSON prints the parsed object:" << std::endl;'
        print '\tstd::cerr << formatted << std::endl;'
        print '\tcJSON_Delete(object);'
        print '\tobject = cJSON_Parse(formatted);'
        print '\tfree(formatted);'
        print '\tASSERT_TRUE(NULL != object);'
        print '\tformatted = cJSON_Print(object);'
        print '\tstd::cerr << "cJSON prints the re-parsed object:" << std::endl;'
        print '\tstd::cerr << formatted << std::endl;'
        print '\tfree(formatted);'
        # Now we walk it through
        for (path, value) in objwalk(json_object):
            print '\t{'
            print '\t\t// ' + format_path(path) + " = '" + safe_string(str(value)) + "'"
            print '\t\tcJSON* child = object;'
            for p in path:
                if (type(p) == int):
                    print '\t\tASSERT_EQ(cJSON_Array, child->type);'
                    print '\t\tchild = cJSON_GetArrayItem(child, ' + str(p) + ');'
                else:
                    print '\t\tASSERT_EQ(cJSON_Object, child->type);'
                    print '\t\tchild = cJSON_GetObjectItem(child, "' + c_string(p) + '");'
                print '\t\tASSERT_TRUE(NULL != child);'
            if (value == None):
                print '\t\tASSERT_EQ(cJSON_NULL, child->type);'
            elif (type(value) == str or type(value) == unicode):
                print '\t\tASSERT_EQ(cJSON_String, child->type);'
                print '\t\tASSERT_STREQ("' + c_string(value) + '", child->valuestring);'
            elif (type(value) == int or type(value) == float):
                print '\t\tASSERT_EQ(cJSON_Number, child->type);'
                if (math.isnan(value) or math.isinf(value)):
                    print '\t\t// Testing for NaN or Infinity is not implemented'
                else:
                    print '\t\tASSERT_FLOAT_EQ(' + str(value) + ', child->valuedouble);'
            elif (type(value) == bool):
                if (value):
                    print '\t\tASSERT_EQ(cJSON_True, child->type);'
                else:
                    print '\t\tASSERT_EQ(cJSON_False, child->type);'
            else:
                print '\t\tASSERT_TRUE(false) << "BUG! CreateGTest.py! value from objwalk has unexpected type ' + type(value).__name__ + '";'
            print '\t}'
        print '\tcJSON_Delete(object);'
    else:
        print '\tASSERT_EQ(NULL, object);'
    print '}'
    print ''

