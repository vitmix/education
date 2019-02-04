#include "pch.h"
#include "testLib\Bubble.h"

TEST(TestOldBubbleSort, EmptyVectorSortA) {
	try {
		std::vector<int> v;
		BubbleSortOld(v);
		SUCCEED();
	}
	catch (const std::exception &ex) {
		FAIL() << ex.what() << "\n";
	}
}

TEST(TestOldBubbleSort, EmptyVectorSortB) {
	std::vector<int> v;
	EXPECT_NO_THROW({ BubbleSortOld(v); });
}

TEST(TestOldBubbleSort, SimpleSort) {
	std::vector<int> v = { 9,4,2,1,3 };
	BubbleSortOld(v);

	EXPECT_EQ(v[0], 1);
	EXPECT_EQ(v[1], 2);
	EXPECT_EQ(v[2], 3);
	EXPECT_EQ(v[3], 4);
	EXPECT_EQ(v[4], 9);
}
/////////////////////////////////////////////////
TEST(TestNewBubbleSort, EmptyVectorSort) {
	std::vector<int> v;
	EXPECT_NO_THROW({ BubbleSortNew(v); });
}

TEST(TestNewBubbleSort, SimpleSort) {
	std::vector<int> vA{ 3,4,1,9,7 };
	std::vector<int> vB{ 1,3,4,7,9 };

	BubbleSortNew(vA);

	EXPECT_EQ(vA, vB);
}

TEST(TestNewBubbleSort, OneElem) {
	std::vector<int> vA{ 3 };

	BubbleSortNew(vA);

	EXPECT_EQ(vA[0], 3);
}

TEST(TestNewBubbleSort, AllTheSame) {
	std::vector<int> vA{ 3,3,3,3,3 };
	std::vector<int> vB{ 3,3,3,3,3 };

	BubbleSortNew(vA);

	ASSERT_TRUE(vA == vB);
}

TEST(TestNewBubbleSort, NegValues) {
	std::vector<int> vA{ -2,-4,-1,-20,-22,-3 };

	BubbleSortNew(vA);

	ASSERT_TRUE(std::is_sorted(vA.begin(), vA.end()));
}

TEST(TestNewBubbleSort, NegAndPosValues) {
	std::vector<int> vA{ -2,-4,1,20,-22,3 };
	std::vector<int> vB{ -22,-4,-2,1,3,20 };

	BubbleSortNew(vA);

	EXPECT_EQ(vA, vB);
}

