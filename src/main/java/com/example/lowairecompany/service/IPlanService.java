package com.example.lowairecompany.service;

import com.example.lowairecompany.pojo.Plan;

public interface IPlanService {
    void add(Plan plan);
    Plan get(Integer id);
    Plan edit(Plan plan);
    Plan delete(Integer id);
}
