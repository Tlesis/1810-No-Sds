package frc.robot;

public enum Position {
    FrontLeft(0, "Front Left"),
    BackLeft(1, "Back Left"),
    FrontRight(2, "Front Right"),
    BackRight(3, "Back Right");

    String name;
    int value;

    Position(int value, String name) {
        this.value = value;
        this.name = name;
    }

    public String getName() {
        return this.name;
    }

    public int getValue() {
        return this.value;
    }
}
