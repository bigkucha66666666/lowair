package com.example.lowairecompany.service;

import com.example.lowairecompany.pojo.Climate;
import java.util.List;

public interface IClimateService {
    void add(Climate climate);
    Climate get(Integer id);
    Climate edit(Climate climate);
    Climate delete(Integer id);
    
    //获取所有天气数据
    List<Climate> getAllClimates();
    
    //批量删除天气数据
    void deleteBatch(List<Integer> ids);
}
