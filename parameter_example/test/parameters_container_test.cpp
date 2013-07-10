#include <vector>
#include "gtest/gtest.h"
#include "parameter_server/parameters_container.h"

class ParametersContainerTest : public testing::Test
{
protected:
  void SetUp()
  {
    cont.set("p", 10);
  }

  ParametersContainer<int> cont;
};

TEST_F(ParametersContainerTest, SetAndGetMany) {
  int out1, out2;
  cont.set("m", 20);
  cont.get("p", out1);
  cont.get("m", out2);
  EXPECT_EQ(10, out1);
  EXPECT_EQ(20, out2);
}

TEST_F(ParametersContainerTest, GetNonExistentParameter) {
  int out1;
  EXPECT_THROW(cont.get("nonExistent", out1), NonExistentParameter);
}

TEST_F(ParametersContainerTest, GetAndSetAnArray) {
  ParametersContainer<std::vector<int> > c;

  std::vector<int> v(10);
  std::vector<int> w;

  for (int i=0; i<10; ++i) {
    v[i] = i*10;
  }

  c.set("m", v);
  c.get("m", w);

  for (int i=0; i<w.size(); ++i) {
    EXPECT_EQ(i*10, w[i]);
  }
}

TEST_F(ParametersContainerTest, HasParameter) {
  EXPECT_TRUE(cont.has("p"));
  EXPECT_FALSE(cont.has("noExist"));
}

TEST_F(ParametersContainerTest, getParamNames) {
  cont.set("m", 20);
  ParametersContainer<std::string>::Keys keys;
  cont.keys(keys);
  EXPECT_EQ(2, keys.size());
  EXPECT_TRUE(keys.find("m") != keys.end());
  EXPECT_TRUE(keys.find("p") != keys.end());
}

TEST_F(ParametersContainerTest, delete) {
  cont.remove("p");
  EXPECT_FALSE(cont.has("p"));
}

TEST_F(ParametersContainerTest, deleteNonExistentDoesNothing) {
  cont.remove("noExist");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
