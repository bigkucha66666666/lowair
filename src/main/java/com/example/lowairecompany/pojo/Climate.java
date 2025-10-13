package com.example.lowairecompany.pojo;

import jakarta.persistence.*;

@Table(name="climate")
@Entity
public class Climate {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name="climate_id")
    private Integer id;
    @Column(name="climate_status")
    private boolean status;
    @Column(name="speed")
    private float speed;
    @Column(name="temperature")
    private float temperature;
    @Column(name="wet")
    private float wet;
    public Integer getId() {
        return id;
    }

    public void setId(Integer id) {
        this.id = id;
    }
    public boolean isStatus() {
        return status;
    }

    public void setStatus(boolean status) {
        this.status = status;
    }
    public float getSpeed() {
        return speed;
    }

    public void setSpeed(float speed) {
        this.speed = speed;
    }
    public float getTemperature() {
        return temperature;
    }

    public void setTemperature(float temperature) {
        this.temperature = temperature;
    }
    public float getWet() {
        return wet;
    }

    public void setWet(float wet) {
        this.wet = wet;
    }

    @Override
    public String toString() {
        return "Climate{" +
                "id=" + id +
                ", status=" + status +
                ", speed=" + speed +
                ", temperature=" + temperature +
                ", wet=" + wet +
                '}';
    }
}
