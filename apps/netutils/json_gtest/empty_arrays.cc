#include <limits.h>
#include "gtest/gtest.h"
extern "C" {
#include "cJSON.h"
}

class EmptyArrays : public ::testing::Test,
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

cJSON* EmptyArrays::object;
char* EmptyArrays::output;

TEST_P(EmptyArrays, ValidPrinting)
{
  const char* json = GetParam();
  EmptyArrays::object = cJSON_Parse(json);
  EmptyArrays::output = cJSON_PrintUnformatted(EmptyArrays::object);
  ASSERT_STREQ(json, EmptyArrays::output);
}

INSTANTIATE_TEST_CASE_P(OnlyField, EmptyArrays, ::testing::Values("{\"a\":[]}"));
INSTANTIATE_TEST_CASE_P(FewFields, EmptyArrays, ::testing::Values("{\"a\":[],\"b\":[]}"));
INSTANTIATE_TEST_CASE_P(ArrayOfEmptyArrays, EmptyArrays, ::testing::Values("{\"a\":[[],[]]}"));
INSTANTIATE_TEST_CASE_P(DeeplyNested, EmptyArrays, ::testing::Values("{\"a\":[[[[[[]]]]]]}"));
INSTANTIATE_TEST_CASE_P(DeeplyNested2, EmptyArrays, ::testing::Values("{\"a\":{\"b\":{\"c\":[]}}}"));
