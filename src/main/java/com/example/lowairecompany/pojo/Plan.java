package com.example.lowairecompany.pojo;

import jakarta.persistence.*;

@Table(name = "Plan")
@Entity
public class Plan {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Integer ID;
    @Column(name="plan_name")
    private String plan_name;
    @Column
    private String ues_zone;
    @Column
    private String algorithm;
    @Column
    private String plan_climate;
    @Column
    private int need_count;
    @Column
    private String plan_status;
    public Integer getID() {
        return ID;
    }

    public void setID(Integer ID) {
        this.ID = ID;
    }
    public String getPlan_name() {
        return plan_name;
    }

    public void setPlan_name(String plan_name) {
        this.plan_name = plan_name;
    }
    public String getUes_zone() {
        return ues_zone;
    }

    public void setUes_zone(String ues_zone) {
        this.ues_zone = ues_zone;
    }
    public String getAlgorithm() {
        return algorithm;
    }

    public void setAlgorithm(String algorithm) {
        this.algorithm = algorithm;
    }
    public String getPlan_climate() {
        return plan_climate;
    }

    public void setPlan_climate(String plan_climate) {
        this.plan_climate = plan_climate;
    }
    public int getNeed_count() {
        return need_count;
    }

    public void setNeed_count(int need_count) {
        this.need_count = need_count;
    }
    public String getPlan_status() {
        return plan_status;
    }

    public void setPlan_status(String plan_status) {
        this.plan_status = plan_status;
    }

    @Override
    public String toString() {
        return "Plan{" +
                "ID=" + ID +
                ", plan_name='" + plan_name + '\'' +
                ", ues_zone='" + ues_zone + '\'' +
                ", algorithm='" + algorithm + '\'' +
                ", plan_climate='" + plan_climate + '\'' +
                ", need_count=" + need_count +
                ", plan_status='" + plan_status + '\'' +
                '}';
    }
}
