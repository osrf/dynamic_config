#include <vector>
#include <iostream>
#include "gtest/gtest.h"
#include "parameter_server/command.h"

class CommandTest : public testing::Test
{
protected:
  // virtual void SetUp()
  // { }
  
  // virtual void TearDown()
  // { }

  class MockClass
  {
  public:
    MockClass()
    : numOfArgsCalled(3, false)
    { }

    void noArgs()
    {
      numOfArgsCalled[0] = true;
    }

    template <class Arg1>
    void oneArg(Arg1 aInt)
    {
      numOfArgsCalled[1] = true;
    }

    template <class Arg1, class Arg2>
    void twoArgs(Arg1 aInt, const Arg2& aString)
    {
      numOfArgsCalled[2] = true;
    }    

    void justOneMethodCalled(int numOfArgs)
    {
      for (int i=0; i<numOfArgsCalled.size(); ++i) {
        if (i == numOfArgs) 
          EXPECT_TRUE(numOfArgsCalled[i]);
        else 
          EXPECT_FALSE(numOfArgsCalled[i]);
      }
    }

  private:
    std::vector<bool> numOfArgsCalled;
  };

  MockClass mock;
};

TEST_F(CommandTest, NoArgsCommand) {
  using namespace parameter_server::command;
  Command c = make_command(&MockClass::noArgs, &mock);
  c();
  mock.justOneMethodCalled(0);
}

TEST_F(CommandTest, OneArgCommand) {
  using namespace parameter_server::command;
  Command c = make_command(&MockClass::oneArg, &mock, 10);
  c();
  mock.justOneMethodCalled(1);
}

TEST_F(CommandTest, TwoArgsCommand) {
  using namespace parameter_server::command;
  Command c = make_command(&MockClass::twoArgs, &mock, 12.12f, 
                           std::string("My string"));
  c();
  mock.justOneMethodCalled(2);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
