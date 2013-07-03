#include <boost/shared_ptr.hpp>
#include "gtest/gtest.h"
#include "parameter_server/subscribers_manager.h"

class SubscribersManagerTest : public testing::Test
{
protected:
  class MyNotifier
  {
    typedef boost::shared_ptr<int> IntPtr;

  public:
    MyNotifier()
    : counter_(new int)
    { *counter_ = 0; }

    bool operator==(const MyNotifier& rhs) const
    { return true; }

    void notify(int event)
    { ++(*counter_); }
      
    int counter()
    { return *counter_; }

  private:
    IntPtr counter_;
  
  };

  MyNotifier notifier;
  parameter_server::SubscribersManager<MyNotifier> manager;
};

TEST_F(SubscribersManagerTest, AddOneNotifier) {
  manager.add("param1", notifier);
  EXPECT_EQ(manager.size("param1"), 1);
}

TEST_F(SubscribersManagerTest, AddTwoNotifiers) {
  manager.add("param1", notifier);
  manager.add("param1", notifier);
  EXPECT_EQ(manager.size("param1"), 2);
}

TEST_F(SubscribersManagerTest, RemoveOneNotifier) {
  manager.add("param1", notifier);
  manager.remove("param1", notifier);
  EXPECT_EQ(manager.size("param1"), 0);  
}

TEST_F(SubscribersManagerTest, RemoveTwoNotifiers) {
  std::string p("param1");

  manager.add(p, notifier);
  manager.add(p, notifier);

  manager.remove(p, notifier);
  manager.remove(p, notifier);
}

TEST_F(SubscribersManagerTest, NotifyOne) {
  std::string p("param1");
  manager.add(p, notifier);
  manager.notify(p, 0);
  EXPECT_EQ(notifier.counter(), 1);
  manager.notify(p, 0);
  EXPECT_EQ(notifier.counter(), 2);
}

TEST_F(SubscribersManagerTest, NotifyMany) {
  std::string p1("param1");
  std::string p2("param2");

  for (int i=0; i<10; ++i) {
    manager.add(p1, notifier);
    manager.add(p2, notifier);
  }
  manager.notify(p1, 0);
  EXPECT_EQ(notifier.counter(), 10);
  manager.notify(p2, 0);
  EXPECT_EQ(notifier.counter(), 20);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
