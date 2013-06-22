#include <string>
#include "ros/gsoc/polymorphic_map.h"
#include <gtest/gtest.h>

typedef ros::gsoc::PolymorphicMap<std::string> ParamWarehouse;

class ParameterContainerSuite : public ::testing::Test
{
protected:
  template < typename T >
  void setAndGetTest(std::string key, const T& input)
  {
    warehouse.set(key, input);
    T output;
    warehouse.get(key, output);
    EXPECT_EQ(input,output);
  }

  ParamWarehouse warehouse;
};

TEST_F(ParameterContainerSuite, getSetTest)
{
  setAndGetTest<std::string>("myString", "My string");
  setAndGetTest<int>("myInt", 10);
  setAndGetTest<float>("myFloat", 10.5f);
  setAndGetTest<double>("myDouble", 10.6);
  setAndGetTest<bool>("myBoolean", true);

  // Override an existing key with value of other type
  setAndGetTest<float>("myInt", 10.0);

  // Invalid Type Exception
  bool exceptionRaised;
  try {
    float shouldBeAString;
    warehouse.get("myString", shouldBeAString);
    exceptionRaised = false;
  } catch (ros::gsoc::InvalidParameterTypeException ex) {
    exceptionRaised = true;
  }
  EXPECT_TRUE(exceptionRaised);
}

TEST_F(ParameterContainerSuite, hasTest)
{
  std::string key("anInt"); 
  warehouse.set(key, 20);
  EXPECT_TRUE(warehouse.has(key));
  EXPECT_FALSE(warehouse.has("notAKey"));
}

TEST_F(ParameterContainerSuite, delTest)
{
  std::string key("myKey");
  warehouse.set(key,20);
  warehouse.del(key);
  EXPECT_FALSE(warehouse.has(key));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

