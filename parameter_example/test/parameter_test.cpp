#include "gtest/gtest.h"
#include "parameter_server/parameter.h"

class ParameterServerTest : public testing::Test
{
protected:
  typedef std::vector<int> Ints;
  typedef int Subscriber;
  typedef parameter_server::Parameter<Ints, Subscriber> MyParameter;

  virtual void SetUp()
  { }

  Ints intsOfSize(int size)
  {
    Ints ints(size);
    for (int i=0; i<size; ++i)
      ints.push_back(i*10);
    return ints;
  }

  MyParameter p;
};

TEST_F(ParameterServerTest, CopyConstructor) {
  MyParameter q(p);
  EXPECT_EQ(p, q);
}

TEST_F(ParameterServerTest, OperatorEqual) {
  MyParameter q = p;
  EXPECT_EQ(p, q);
}

TEST_F(ParameterServerTest, CanGet) {
  p.setValue(intsOfSize(10));
  Ints ints2;
  p.getValue(ints2);
  EXPECT_EQ(intsOfSize(10), ints2);
}

TEST_F(ParameterServerTest, HasValueFalse) {
  EXPECT_FALSE(p.hasValue());
}

TEST_F(ParameterServerTest, HasValueTrue) {
  p.setValue(intsOfSize(10));
  EXPECT_TRUE(p.hasValue());
}

TEST_F(ParameterServerTest, DeleteValueNotSetted) {
  p.deleteValue();
  EXPECT_FALSE(p.hasValue());
}

TEST_F(ParameterServerTest, DeleteValueSetted) {
  p.setValue(intsOfSize(10));
  p.deleteValue();
  EXPECT_FALSE(p.hasValue());
}

TEST_F(ParameterServerTest, CheckSubscribersEmpty) {
  EXPECT_TRUE(p.subscribersEmpty());
  EXPECT_EQ(0, p.subscribersSize());
}

TEST_F(ParameterServerTest, CheckSubscribersNotEmpty) {
  p.addSubscriber(0);
  EXPECT_FALSE(p.subscribersEmpty());
  EXPECT_EQ(1, p.subscribersSize());
}

TEST_F(ParameterServerTest, IterateThroughSusbcribers) {
  for (int i=0; i<10; ++i)
    p.addSubscriber(i);
  MyParameter::iterator it = p.subscribersBegin();
  MyParameter::iterator end = p.subscribersEnd();
  int i = 0;
  for (; it != end; ++it)
    EXPECT_EQ(i++, *it);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
