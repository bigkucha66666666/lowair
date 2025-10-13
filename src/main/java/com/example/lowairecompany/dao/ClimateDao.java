package com.example.lowairecompany.dao;

import com.example.lowairecompany.pojo.Climate;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface ClimateDao extends CrudRepository<Climate, Integer> {
}
