package com.example.lowairecompany.controller;

import com.example.lowairecompany.pojo.Plan;
import com.example.lowairecompany.pojo.ResponseMessage;
import com.example.lowairecompany.service.IPlanService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/plan")
public class PlanController {

    @Autowired
    private IPlanService planService;

    @PostMapping
    public ResponseMessage<Void> add(@RequestBody Plan plan) {
        planService.add(plan);
        return ResponseMessage.success(null);
    }

    @GetMapping("/{id}")
    public ResponseMessage<Plan> get(@PathVariable Integer id) {
        return ResponseMessage.success(planService.get(id));
    }

    @PutMapping
    public ResponseMessage<Plan> edit(@Validated @RequestBody Plan plan) {
        return ResponseMessage.success(planService.edit(plan));
    }

    @DeleteMapping("/{id}")
    public ResponseMessage<Plan> delete(@PathVariable Integer id) {
        return ResponseMessage.success(planService.delete(id));
    }
}
