#include <map>
#include <boost/shared_ptr.hpp>
#include "gtest/gtest.h"
#include "parameter_server/subscribers_manager.h"

class SubscribersManagerTest : public testing::Test
{
  protected:
  virtual void SetUp()
  {
    subs.add("param1", "service1");
    subs.add("param1", "service2");
  }

  class MockNotifier
    {
    public:
    MockNotifier ()
    : enabled(true)
    { }

    int timesCalled(const std::string& param, const std::string& subscriber)
    {
      return timesSubsCalled[param + subscriber];
    }

    void enable(bool enabled)
    {
      this->enabled = enabled;
    }

    protected:
    bool call(const std::string& param, const std::string& subscriber, int event)
    {
      if (enabled)
        timesSubsCalled[param + subscriber] += 1;
      return enabled;
    }

    private:
      std::map<std::string, int> timesSubsCalled;
      bool enabled;
  };

  parameter_server::SubscribersManager<MockNotifier> subs;
};

TEST_F(SubscribersManagerTest, NotifyMany) {
  subs.add("param2", "service1");
  subs.add("param2", "service2");
  subs.notify("param1", 0);
  subs.notify("param2", 0);
  subs.notify("param2", 0);
  EXPECT_EQ(1, subs.timesCalled("param1", "service1"));
  EXPECT_EQ(1, subs.timesCalled("param1", "service2"));
  EXPECT_EQ(2, subs.timesCalled("param2", "service1"));
  EXPECT_EQ(2, subs.timesCalled("param2", "service2"));
}

TEST_F(SubscribersManagerTest, DontDuplicate) {
  subs.add("param1", "service1");
  subs.notify("param1", 0);
  EXPECT_EQ(1, subs.timesCalled("param1", "service1"));
}

TEST_F(SubscribersManagerTest, RemoveOneButNotifyOthers) {
  subs.remove("param1", "service1");
  subs.notify("param1", 0);
  EXPECT_EQ(0, subs.timesCalled("param1", "service1"));
  EXPECT_EQ(1, subs.timesCalled("param1", "service2"));
}

TEST_F(SubscribersManagerTest, RemoveNotExistent_NoFail) {
  subs.remove("no_param", "no_srv");
}

TEST_F(SubscribersManagerTest, RemoveAfter3Tries) {
  subs.tries(3);
  subs.enable(false);
  subs.notify("param1", 0);
  subs.notify("param1", 0);
  subs.notify("param1", 0);
  subs.enable(true);
  subs.notify("param1", 0);
  EXPECT_EQ(0, subs.timesCalled("param1", "service1"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
