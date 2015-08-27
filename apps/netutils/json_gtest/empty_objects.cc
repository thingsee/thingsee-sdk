#include <limits.h>
#include "gtest/gtest.h"
extern "C" {
#include "cJSON.h"
}

class EmptyObjects : public ::testing::Test,
                    public ::testing::WithParamInterface<const char*>
{
public:
  static cJSON* object;
  static char* output;
public:
  static void SetUpTestCase()
  {
    object = NULL;
    output = NULL;
  }
  static void TearDownTestCase()
  {
    cJSON_Delete(object);
    free(output);
  }
};

cJSON* EmptyObjects::object;
char* EmptyObjects::output;

TEST_P(EmptyObjects, ValidPrinting)
{
  const char* json = GetParam();
  EmptyObjects::object = cJSON_Parse(json);
  EmptyObjects::output = cJSON_PrintUnformatted(EmptyObjects::object);
  ASSERT_STREQ(json, EmptyObjects::output);
}

INSTANTIATE_TEST_CASE_P(OnlyField, EmptyObjects, ::testing::Values("{\"a\":{}}"));
INSTANTIATE_TEST_CASE_P(FewFields, EmptyObjects, ::testing::Values("{\"a\":{},\"b\":{}}"));
INSTANTIATE_TEST_CASE_P(ArrayOfEmptyObjects, EmptyObjects, ::testing::Values("{\"a\":[{},{},{}]}"));
INSTANTIATE_TEST_CASE_P(DeeplyNested, EmptyObjects, ::testing::Values("{\"a\":{\"b\":{\"c\":{}}}}"));
