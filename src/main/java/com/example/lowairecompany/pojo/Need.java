package com.example.lowairecompany.pojo;
import jakarta.persistence.*;
@Table(name="needs")
@Entity
public class Need {
    @jakarta.persistence.Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "need_ID")
    private Integer ID;
    @Column(name = "need_Name")
    private String Name;
    @Column(name = "need_air_Zone")
    private String airZone;
    @Column(name = "need_status")
    private String status;
    @Column(name = "up_time")
    private Integer up_time;
    @Column(name = "need_height")
    private String height;
    public Integer getID() {
        return ID;
    }

    public void setID(Integer ID) {
        this.ID = ID;
    }
    public String getName() {
        return Name;
    }

    public void setName(String name) {
        Name = name;
    }
    public String getAirZone() {
        return airZone;
    }

    public void setAirZone(String airZone) {
        this.airZone = airZone;
    }
    public String getStatus() {
        return status;
    }

    public void setStatus(String status) {
        this.status = status;
    }
    public Integer getUp_time() {
        return up_time;
    }

    public void setUp_time(Integer up_time) {
        this.up_time = up_time;
    }
    public String getHeight() {
        return height;
    }
    public void setHeight(String height) {
        this.height = height;
    }
    @Override
    public String toString() {
        return "Need{" +
                "ID=" + ID +
                ", Name='" + Name + '\'' +
                ", airZone='" + airZone + '\'' +
                ", status='" + status + '\'' +
                ", up_time=" + up_time +
                ", height='" + height + '\'' +
                '}';
    }
}
