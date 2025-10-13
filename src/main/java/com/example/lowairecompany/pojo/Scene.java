package com.example.lowairecompany.pojo;

import jakarta.persistence.*;

@Table(name="Scene")
@Entity
public class Scene {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name="scene_ID")
    private Integer ID;
    @Column(name="scene_name")
    private String scene_name;
    @Column(name="airzone_name")
    private String airzone_name;
    @Column(name="event")
    private String event;
    public Integer getID() {
        return ID;
    }

    public void setID(Integer ID) {
        this.ID = ID;
    }
    public String getScene_name() {
        return scene_name;
    }

    public void setScene_name(String scene_name) {
        this.scene_name = scene_name;
    }
    public String getAirzone_name() {
        return airzone_name;
    }

    public void setAirzone_name(String airzone_name) {
        this.airzone_name = airzone_name;
    }
    public String getEvent() {
        return event;
    }

    public void setEvent(String event) {
        this.event = event;
    }
    @Override
    public String toString() {
        return "Scene{" +
                "ID=" + ID +
                ", scene_name='" + scene_name + '\'' +
                ", airzone_name='" + airzone_name + '\'' +
                ", event='" + event + '\'' +
                '}';
    }
}
