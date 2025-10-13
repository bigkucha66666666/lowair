package com.example.lowairecompany.pojo;

import jakarta.persistence.*;

@Table(name = "airfiled")
@Entity
public class Airfiled {
    @jakarta.persistence.Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name="af-id")
    private Integer Id;
    @Column(name="af_name")
    private String Name;
    @Column(name="status")
    private String Status;
    @Column(name="type")
    private String Typr;
    @Column(name="stock")
    private int Stock;
    public Integer getId() {
        return Id;
    }
    public void setId(Integer id) {
        Id = id;
    }
    public String getName() {
        return Name;
    }
    public void setName(String name) {
        Name = name;
    }
    public String getStatus() {
        return Status;
    }
    public void setStatus(String status) {
        Status = status;
    }
    public String getTypr() {
        return Typr;
    }
    public void setTypr(String typr) {
        Typr = typr;
    }
    public int getStock() {
        return Stock;
    }
    public void setStock(int stock) {
        Stock = stock;
    }
    @Override
    public String toString() {
        return "Airfiled{" +
                "Id=" + Id +
                ", Name='" + Name + '\'' +
                ", Status='" + Status + '\'' +
                ", Typr='" + Typr + '\'' +
                ", Stock=" + Stock +
                '}';
    }
}
