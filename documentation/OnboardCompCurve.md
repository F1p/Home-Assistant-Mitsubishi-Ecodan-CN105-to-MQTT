# Using the onboard Compensation Curve



### About
The software has the ability to store a custom compensation curve onboard - this overcomes some limitations with the Ecodan and allows advanced features to be input.

* The Ecodan Main Room Controller (MRC) has the ability to design a curve with a maximum of 3 points, there is no limit to the number of points which can be sent to the CN105 Ecodan Bridge device.
* There is also no ability to add an +/- offset to the current curve remotely, only by adjusting on the the controller screen. By moving the curve onto the CN105 Ecodan Bridge, an offset can be adjusted remotely.
* The Ecodan uses the Outside Air Temperature (OAT) sensor located on the back of the outdoor unit, this sensor can be influenced by the likes of direct sunlight. By moving the curve onto the CN105 Ecodan Bridge it's possible to use another temperature source to determine the flow temperature.
* The Ecodan cannot automatically adjust for other influencing factors, such as heat loss due to wind or solar gain. By moving the curve onto the CN105 Ecodan Bridge it's possible to automate the offset when conditions may influence the building.


The device can continue to calculate and input to the curve if the connection is broken.



### How to Design, Modify and Operate

The Ecodan is to be placed in Fixed Flow mode, as the value is calculated by the device it will then adjust the flow temperature setpoint automatically which recreates the method the Ecodan uses.
By activating the onboard compensation curve, if not already in Fixed Flow the software will change your Ecodan to this mode.



### Designing a Curve

Similar to the main controller screen, there is a flow temperature and an outside temperature:
![Comp Curve Graph](https://github.com/F1p/Mitsubishi-CN105-Protocol-Decode/blob/master/documentation/images/Curve_Points.png)

The corrosponding data points are:

| Outside Temperature (C) | Flow Temperature (C) |
|--------|---------|
| -10 | 60 |
| 0 | 40 |
| +5 | 35 |
| +10 | 27 |
| +15 | 32 |



The design for Zone1 can be different to Zone2, if your system has suitable Complex 2 Zone hardware (i.e. mixing) to support different flow temperatures on different zones

The Curve is recorded and sent as JSON to be saved on the device, using the example above, the JSON would look like so:

```javascript
{
  "base": {
    "zone1": {
      "curve": [
        {
          "flow": 60,
          "outside": -10
        },
        {
          "flow": 40,
          "outside": 0
        },
        {
          "flow": 35,
          "outside": 0
        },
        {
          "flow": 27,
          "outside": 10
        },
        {
          "flow": 32,
          "outside": 15
        }
      ]
    },
    "zone2": {
      "curve": [
        {
          "flow": 60,
          "outside": -10
        },
        {
          "flow": 40,
          "outside": 0
        },
        {
          "flow": 35,
          "outside": 0
        },
        {
          "flow": 27,
          "outside": 10
        },
        {
          "flow": 32,
          "outside": 15
        }
      ]
    }
  }
}
```


### Programming or Modifying the Curve

Using MQTT Explorer, we can send our design, in JSON to the topic: Ecodan/ASHP/Command/System/CompCurve



```javascript
{
  "base": {
    "zone1": {
      "curve": [
        {
          "flow": 60,
          "outside": -10
        },
        {
          "flow": 35,
          "outside": 0
        },
        {
          "flow": 20,
          "outside": 15
        },
        {
          "flow": 10,
          "outside": 20
        }
      ]
    },
    "zone2": {
      "curve": [
        {
          "flow": 60,
          "outside": -10
        },
        {
          "flow": 35,
          "outside": 0
        },
        {
          "flow": 20,
          "outside": 15
        }
      ]
    }
  }
}
```

![Publish Config](https://github.com/F1p/Mitsubishi-CN105-Protocol-Decode/blob/master/documentation/images/Publish_Curve.png)




You can verify the curve has been saved, looking at the Ecodan/ASHP/Status/CompCurve topic:
![Config Readback](https://github.com/F1p/Mitsubishi-CN105-Protocol-Decode/blob/master/documentation/images/Curve_Status.png)





### Enable or Disable the Mode 

```javascript
{   
    "zone1": {
        "active": true
    }
}
```



### Sending Offsets & Outdoor Temperatures

In a similar way, you can make modification to the Offsets (+/- of the calculated Flow Setpoint) by sending JSON: 

Up to three Offsets exist, all act in the same way by adding or subtracting to the Flow Setpoint calculated on the base curve.
* Manual Offset is intended for users to adjust via a Dashboard
* Wind Offset is intended if you require additional heat at higher windspeed, usually calculated from an automation
* Temp Offset is intended if you reduce flow when solar gain is high, usually calculated from an automation


```javascript
{   
    "zone1": {
        "manual_offset": 0,
        "wind_offset": 1,
        "temp_offset": -2
  }
}
```

![Publish Parameters](https://github.com/F1p/Mitsubishi-CN105-Protocol-Decode/blob/master/documentation/images/Partial_Publish_Curve.png)



The value of Outside Air Temperature (OAT) used in the curve calculation by default is from the sensor located on the unit itself, however you can choose to use your own value for Outdoor Air Temperature in the following way:


Set Use Local Outdoor as False:
```javascript
{   
    "use_local_outdoor": false
}
```

Provide a regularly updating Outdoor Air Temperature Value:
```javascript
{   
    "cloud_outdoor": 14.3
}
```    
![Publish Parameters](https://github.com/F1p/Mitsubishi-CN105-Protocol-Decode/blob/master/documentation/images/CompCurve_Automation.png)


```yaml
alias: Publish Temperature from HA Weather to Bridge
description: ""
triggers:
  - trigger: state
    entity_id:
      - weather.forecast_home
    attribute: temperature
conditions: []
actions:
  - action: mqtt.publish
    metadata: {}
    data:
      evaluate_payload: false
      qos: "0"
      topic: Ecodan/ASHP/Command/System/CompCurve
      payload: >-
        { "cloud_outdoor":{{ state_attr('weather.forecast_home','temperature')
        }} }
mode: single
```


![Publish Parameters](https://github.com/F1p/Mitsubishi-CN105-Protocol-Decode/blob/master/documentation/images/Data_Transfer.png)





If the MQTT connection is lost, the Bridge device will revert to using the local sensor.

NOTE: The FTC may still use the OAT sensor in it's processes, such as defrosting or frequency limiting - this value cannot influence those actions but only determining a Flow Temperature
