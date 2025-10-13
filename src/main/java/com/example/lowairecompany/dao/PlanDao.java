package com.example.lowairecompany.dao;

import com.example.lowairecompany.pojo.Plan;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface PlanDao extends CrudRepository<Plan, Integer> {
}
