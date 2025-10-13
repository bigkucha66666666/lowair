package com.example.lowairecompany.pojo;

import jakarta.persistence.*;

@Table(name="Event")
@Entity
public class Event {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name="event_ID")
    private Integer ID;
    @Column(name="event_name")
    private String event_name;
    @Column(name="event_type")
    private String event_type;
    @Column(name="event_strength")
    private String event_strength;
    @Column(name="strategy")
    private String strategy;
    public Integer getID() {
        return ID;
    }
    public void setID(Integer ID) {
        this.ID = ID;
    }
    public String getEvent_name() {
        return event_name;
    }

    public void setEvent_name(String event_name) {
        this.event_name = event_name;
    }

    public String getEvent_type() {
        return event_type;
    }

    public void setEvent_type(String event_type) {
        this.event_type = event_type;
    }
    public String getEvent_strength() {
        return event_strength;
    }

    public void setEvent_strength(String event_strength) {
        this.event_strength = event_strength;
    }
    public String getStrategy() {
        return strategy;
    }

    public void setStrategy(String strategy) {
        this.strategy = strategy;
    }

    @Override
    public String toString() {
        return "Event{" +
                "ID=" + ID +
                ", event_name='" + event_name + '\'' +
                ", event_type='" + event_type + '\'' +
                ", event_strength='" + event_strength + '\'' +
                ", strategy='" + strategy + '\'' +
                '}';
    }
}
