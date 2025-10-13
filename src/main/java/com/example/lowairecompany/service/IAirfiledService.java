package com.example.lowairecompany.service;

import com.example.lowairecompany.pojo.Airfiled;
import java.util.List;

public interface IAirfiledService {
    void add(Airfiled airfiled);
    Airfiled get(Integer id);
    Airfiled edit(Airfiled airfiled);
    Airfiled delete(Integer id);
    
    //获取所有机场名称
    List<String> getAllAirfiledNames();
    
    //获取所有机场数据
    List<Airfiled> getAllAirfileds();
    
    //批量删除机场
    void deleteBatch(List<Integer> ids);
}
