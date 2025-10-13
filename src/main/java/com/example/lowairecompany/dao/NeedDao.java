package com.example.lowairecompany.dao;

import com.example.lowairecompany.pojo.Need;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface NeedDao extends CrudRepository<Need, Integer> {
}


