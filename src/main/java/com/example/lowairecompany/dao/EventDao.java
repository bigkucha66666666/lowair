package com.example.lowairecompany.dao;

import com.example.lowairecompany.pojo.Event;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface EventDao extends CrudRepository<Event,Integer>{
}
