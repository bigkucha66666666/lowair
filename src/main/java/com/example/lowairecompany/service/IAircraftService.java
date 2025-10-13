package com.example.lowairecompany.service;

import com.example.lowairecompany.pojo.Aircraft;
import java.util.List;

public interface IAircraftService {
    void add(Aircraft aircraft);
    Aircraft get(Integer id);
    Aircraft edit(Aircraft aircraft);
    Aircraft delete(Integer id);
    
    //获取所有飞行器类型
    List<String> getAllAircraftTypes();
    
    //获取所有飞行器数据
    List<Aircraft> getAllAircrafts();
    
    //批量删除飞行器
    void deleteBatch(List<Integer> ids);
}
