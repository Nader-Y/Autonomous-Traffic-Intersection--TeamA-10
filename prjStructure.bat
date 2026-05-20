@echo off
REM =====================================================
REM Autonomous Traffic Intersection Management Project
REM Repository Structure Generator
REM =====================================================

set ROOT=traffic-intersection-system

echo Creating project structure...

REM Root
mkdir %ROOT%

REM =====================================================
REM Project Management (NEW)
REM =====================================================

mkdir %ROOT%\project_management
mkdir %ROOT%\project_management\planning
mkdir %ROOT%\project_management\meetings
mkdir %ROOT%\project_management\risks
mkdir %ROOT%\project_management\milestones
mkdir %ROOT%\project_management\backlog

REM =====================================================
REM Documentation
REM =====================================================

mkdir %ROOT%\docs
mkdir %ROOT%\docs\requirements
mkdir %ROOT%\docs\architecture
mkdir %ROOT%\docs\verification
mkdir %ROOT%\docs\reports
mkdir %ROOT%\docs\presentation

REM =====================================================
REM Models
REM =====================================================

mkdir %ROOT%\models
mkdir %ROOT%\models\uml
mkdir %ROOT%\models\sysml
mkdir %ROOT%\models\uppaal
mkdir %ROOT%\models\queue_models

REM =====================================================
REM Embedded
REM =====================================================

mkdir %ROOT%\embedded
mkdir %ROOT%\embedded\freertos
mkdir %ROOT%\embedded\drivers
mkdir %ROOT%\embedded\scheduler
mkdir %ROOT%\embedded\tests

REM =====================================================
REM Simulation
REM =====================================================

mkdir %ROOT%\simulation
mkdir %ROOT%\simulation\traffic
mkdir %ROOT%\simulation\congestion
mkdir %ROOT%\simulation\metrics

REM =====================================================
REM Hardware
REM =====================================================

mkdir %ROOT%\hardware
mkdir %ROOT%\hardware\modelsim
mkdir %ROOT%\hardware\arbitration_logic

REM =====================================================
REM Utility Folders
REM =====================================================

mkdir %ROOT%\scripts
mkdir %ROOT%\results

REM =====================================================
REM Files
REM =====================================================

type nul > %ROOT%\README.md

type nul > %ROOT%\docs\requirements\requirements_specification.md
type nul > %ROOT%\docs\architecture\system_architecture.md
type nul > %ROOT%\docs\verification\verification_plan.md
type nul > %ROOT%\docs\reports\progress_report.md
type nul > %ROOT%\docs\presentation\presentation_outline.md

type nul > %ROOT%\models\uppaal\intersection_model.xml

type nul > %ROOT%\embedded\freertos\main.c
type nul > %ROOT%\embedded\scheduler\scheduler.c

type nul > %ROOT%\simulation\traffic\traffic_simulation.py

echo.
echo ==========================================
echo Project structure created successfully!
echo ==========================================

pause