#include "gtest/gtest.h"
#include "parameter_server/command.h"
#include "parameter_server/commands_queue.h"


class CommandsQueueTest : public testing::Test
{
protected:
  
  class Counter
  {
  public:
    Counter()
    : result_(0) { }

    void count()
    { ++result_; }

    int result() 
    { return result_; }

  private:
    int result_;
  };

};

TEST_F(CommandsQueueTest, ExecuteOneCommand) {
  using namespace parameter_server::command;
  CommandsQueue<Command> cq;
  Counter counter;

  cq.add(make_command(&Counter::count, &counter));
  cq.executeAll();

  EXPECT_EQ(counter.result(), 1);
  EXPECT_EQ(cq.size(), 0);
}

TEST_F(CommandsQueueTest, ExecuteTwoCommands) {
  using namespace parameter_server::command;
  CommandsQueue<Command> cq;
  Counter counter;

  cq.add(make_command(&Counter::count, &counter));
  cq.add(make_command(&Counter::count, &counter));
  cq.executeAll();

  EXPECT_EQ(counter.result(), 2);
  EXPECT_EQ(cq.size(), 0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
