package com.example.lowairecompany.pojo;

import jakarta.persistence.*;

@Table(name="Aircraft")
@Entity
public class Aircraft {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name="aircraft_id")
    private Integer ID;
    @Column(name="type")
    private String craft_type;
    @Column(name="manufacture")
    private String manufacture;
    @Column(name="wheelbase")
    private float wheelbase;
    @Column(name="milege")
    private int mileage;
    @Column(name="radius")
    private float radius;
    @Column(name="fly_speed")
    private float fly_speed;
    @Column(name="wind_speed")
    private float wind_speed;
    @Column(name="lasttime")
    private int lasttime;
    public Integer getID() {
        return ID;
    }
    public void setID(Integer ID) {
        this.ID = ID;
    }
    public String getCraft_type() {
        return craft_type;
    }

    public void setCraft_type(String craft_type) {
        this.craft_type = craft_type;
    }
    public String getManufacture() {
        return manufacture;
    }

    public void setManufacture(String manufacture) {
        this.manufacture = manufacture;
    }
    public float getWheelbase() {
        return wheelbase;
    }

    public void setWheelbase(float wheelbase) {
        this.wheelbase = wheelbase;
    }
    public int getMileage() {
        return mileage;
    }

    public void setMileage(int mileage) {
        this.mileage = mileage;
    }
    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = radius;
    }
    public float getFly_speed() {
        return fly_speed;
    }

    public void setFly_speed(float fly_speed) {
        this.fly_speed = fly_speed;
    }
    public float getWind_speed() {
        return wind_speed;
    }

    public void setWind_speed(float wind_speed) {
        this.wind_speed = wind_speed;
    }
    public int getLasttime() {
        return lasttime;
    }
    public void setLasttime(int lasttime) {
        this.lasttime = lasttime;
    }
}
