package com.example.lowairecompany.pojo;

import jakarta.persistence.*;
@Table(name="airZone")
@Entity
public class AirZone {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name="zone_id")
    private Integer ZoneId;
    @Column(name="zone_name")
    private String zoneName;
    @Column(name="zone_stock")
    private String zoneStock;
    @Column(name="granularity")
    private Integer granularity;
    @Column(name="height")
    private float height;
    public Integer getZoneId() {
        return ZoneId;
    }

    public void setZoneId(Integer zoneId) {
        ZoneId = zoneId;
    }

    public String getZoneName() {
        return zoneName;
    }

    public String getZoneStock() {
        return zoneStock;
    }

    public void setZoneStock(String zoneStock) {
        this.zoneStock = zoneStock;
    }

    public Integer getGranularity() {
        return granularity;
    }

    public void setGranularity(Integer granularity) {
        this.granularity = granularity;
    }

    public void setZoneName(String zoneName) {
        this.zoneName = zoneName;
    }

    public float getHeight() {
        return height;
    }

    public void setHeight(float height) {
        this.height = height;
    }
}
