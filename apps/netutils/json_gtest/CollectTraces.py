import json as JSON

def convert_action(action):
    return action.__name__

def convert_actions(actions):
    return map(convert_action, actions)

def extract_object_actions(obj):
    return obj['actions']

def extract_action(trace):
   return trace[0]

def extract_testcase_actions(testcase):
    return map(extract_action, testcase)

def extract_final_json(testcase):
    return testcase[-1][-1]

def load_json(json):
    try:
        obj = JSON.loads(json)
        return obj
    except ValueError:
        return None

def find_unique(from_modules):
    unique = []
    for module in from_modules:
        for testcase in module.testSuite:
            actions = extract_testcase_actions(testcase)
            named_actions = convert_actions(actions)
            if (not named_actions in map(extract_object_actions, unique)):
                json = extract_final_json(testcase)
                obj = {
                    'json': json,
                    'object': load_json(json),
                    'actions': named_actions
                    }
                unique.append(obj)
    return unique

