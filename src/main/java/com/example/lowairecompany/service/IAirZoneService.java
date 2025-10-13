package com.example.lowairecompany.service;

import com.example.lowairecompany.pojo.AirZone;
import java.util.List;

public interface IAirZoneService {
    void add(AirZone airZone);
    AirZone get(Integer zoneId);
    AirZone edit(AirZone airZone);
    AirZone delete(Integer zoneId);
    
    //获取所有空域名称
    List<String> getAllZoneNames();
    
    //获取所有空域数据
    List<AirZone> getAllZones();
    
    //批量删除空域
    void deleteBatch(List<Integer> zoneIds);
}
