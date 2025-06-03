#include <unity.h>

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_test() {
    TEST_ASSERT_TRUE(true);
}

int main( int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_test);

    UNITY_END();
}