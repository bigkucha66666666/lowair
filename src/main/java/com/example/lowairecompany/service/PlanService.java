package com.example.lowairecompany.service;

import com.example.lowairecompany.dao.PlanDao;
import com.example.lowairecompany.pojo.Plan;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class PlanService implements IPlanService {

    @Autowired
    private PlanDao planDao;

    @Override
    public void add(Plan plan) {
        planDao.save(plan);
    }

    @Override
    public Plan get(Integer id) {
        return planDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Plan 不存在"));
    }

    @Override
    public Plan edit(Plan plan) {
        return planDao.save(plan);
    }

    @Override
    public Plan delete(Integer id) {
        Plan existing = planDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Plan 不存在"));
        planDao.deleteById(id);
        return existing;
    }
}
