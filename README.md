# ROS-I Workcell Tools for RMF
This is a collection of packages to aid in the deployment of workcells (i.e ingestors and dispensers) in RMF.

## Supported System
* ROS2 Humble

## Packages
* [workcell triggers](#workcell-triggers)
* [pseudo workcells](#pseudo-workcells)

### Workcell Triggers
This package contains a library which users can use to trigger a function during dispensing and ingestion in a RMF delivery task.

For example to create a dispenser trigger:
* create your custom callback function of type `std::function<void (const std::string &)>`{:.cpp}

* create a ROS node
* call `make_dispenser(rclcpp:Node::SharedPtr node, std::function<void (const std::string &)> func)`{:.cpp} method and pass the ROS node and your custom trigger function.
```cpp
std::function<void (const std::string &)> func =
  [](const std::string & robot_id)
  {
    // Do something
  };
std::shared_ptr<workcell_triggers::DispenserTrigger> dispenser_trigger =
  workcell_triggers::DispenserTrigger::make_dispenser_trigger(node, func);
```

The function would be triggered upon receiving the [`dispenser_request`](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_dispenser_msgs/msg/DispenserRequest.msg) or [`ingestor_request`](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_ingestor_msgs/msg/IngestorRequest.msg) with the matching `target_guid`
<br></br>

### Pseudo Workcells
The pseudo workcells allows for the convenience of launching of multiple "psuedo" dispensers and ingestors. These "pseudo" ingestors and dispensers are simply `workcell_triggers::DispenserTrigger` and `workcell_triggers::IngestorTrigger` objects that execute nothing but a simple log using `RCLCPP_INFO()` in its callback function.

The list of ingestors and dispensers that will be craeted can be configured in the launch file:
```python
ingestor_list = [
    'product_dropoff_1',
    'product_dropoff_2',
    'stock_holding_1',
    'stock_holding_2',
    'stock_holding_3',
    'stock_holding_4',
]

dispenser_list = [
    'storage_1_dispenser',
    'storage_2_dispenser',
    'cnc_1_dispenser',
    'cnc_2_dispenser',
    'cnc_3_dispenser'
]
```
