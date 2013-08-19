#include "aseta_task_management/TaskManager.h"
#include <gtest/gtest.h>

// Declare a test
TEST(TaskManagerSuite, unitTest)
{
	EXPECT_TRUE(false);
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}