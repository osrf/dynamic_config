#include "gtest/gtest.h"
#include "parameter_server/parameter.h"

class ParameterServerTest : public testing::Test
{
protected:
  typedef std::vector<int> Ints;

  parameter_server::Parameter<Ints> p;
};

TEST_F(ParameterServerTest, CopyConstructor) {
  parameter_server::Parameter<Ints> q(p);
  EXPECT_EQ(p, q);
}

TEST_F(ParameterServerTest, OperatorEqual) {
  parameter_server::Parameter<Ints> q = p;
  EXPECT_EQ(p, q);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}