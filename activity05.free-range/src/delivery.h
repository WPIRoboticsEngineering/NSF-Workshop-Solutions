#pragma once

enum Road {ROAD_MAIN, ROAD_PICKUP, ROAD_ABC, ROAD_A, ROAD_A_DROP, ROAD_B};
enum Destination {NONE, START, PICKUP, HOUSE_A, HOUSE_B, HOUSE_C};

struct Delivery
{
    // The destination for the delivery
    Destination deliveryDest = NONE;

    // The current destination (e.g., pickup, back to start, or a house)
    Destination currDest = NONE;

    // The current location
    Road currLocation = ROAD_MAIN;
};
