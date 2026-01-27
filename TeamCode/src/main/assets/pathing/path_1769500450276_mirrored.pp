{
  "startPoint": {
    "x": 110.5,
    "y": 135.5,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 0,
    "locked": false
  },
  "lines": [
    {
      "id": "line-b5ldzl3omka",
      "name": "1. Start -> Score Pre-load",
      "endPoint": {
        "x": 90,
        "y": 87,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 315
      },
      "controlPoints": [
        {
          "x": 96,
          "y": 105
        }
      ],
      "color": "#6CB966",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8ati4-s3o6ay",
      "name": "2. Score -> Pickup 1 ",
      "endPoint": {
        "x": 129,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 315,
        "endDeg": 0
      },
      "controlPoints": [
        {
          "x": 105,
          "y": 87
        }
      ],
      "color": "#BC657D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8dhdy-mcnxsg",
      "name": "3. Pickup 1 -> Score",
      "endPoint": {
        "x": 90,
        "y": 87,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 315
      },
      "controlPoints": [],
      "color": "#675557",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8ebma-ttkvv6",
      "name": "4. Score -> Pickup 2 ",
      "endPoint": {
        "x": 129,
        "y": 60,
        "heading": "linear",
        "reverse": false,
        "startDeg": 315,
        "endDeg": 0
      },
      "controlPoints": [
        {
          "x": 91,
          "y": 57
        }
      ],
      "color": "#7A8DDD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8g427-vmhfb5",
      "name": "5. Pickup 2 -> Score",
      "endPoint": {
        "x": 90,
        "y": 87,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 315
      },
      "controlPoints": [],
      "color": "#98DC7D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8gogm-cmjan5",
      "name": "6. Score -> Pickup 3 ",
      "endPoint": {
        "x": 133,
        "y": 35,
        "heading": "linear",
        "reverse": false,
        "startDeg": 315,
        "endDeg": 0
      },
      "controlPoints": [
        {
          "x": 96.04656612487787,
          "y": 21.02328306243894
        },
        {
          "x": 129.02328306243894,
          "y": 37
        }
      ],
      "color": "#98D69A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8lqj5-10p18g",
      "name": "7. Pickup 3 -> Score",
      "endPoint": {
        "x": 90,
        "y": 87,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 315
      },
      "controlPoints": [],
      "color": "#D768A8",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8p6oy-u6t2fn",
      "name": "8.  Park",
      "endPoint": {
        "x": 90,
        "y": 57,
        "heading": "linear",
        "reverse": false,
        "startDeg": 315,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#6DABAA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 24,
          "y": 144
        },
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 137,
          "y": 119
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 137,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-b5ldzl3omka"
    },
    {
      "kind": "wait",
      "id": "mkw8rg5a-j51nap",
      "name": "Wait",
      "durationMs": 4000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkw8ati4-s3o6ay"
    },
    {
      "kind": "path",
      "lineId": "mkw8dhdy-mcnxsg"
    },
    {
      "kind": "wait",
      "id": "mkw8ry34-dnss6s",
      "name": "Wait",
      "durationMs": 4000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkw8ebma-ttkvv6"
    },
    {
      "kind": "path",
      "lineId": "mkw8g427-vmhfb5"
    },
    {
      "kind": "wait",
      "id": "mkw8sbp6-9fde8f",
      "name": "Wait",
      "durationMs": 4000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkw8gogm-cmjan5"
    },
    {
      "kind": "path",
      "lineId": "mkw8lqj5-10p18g"
    },
    {
      "kind": "path",
      "lineId": "mkw8p6oy-u6t2fn"
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-01-27T07:55:17.825Z"
}